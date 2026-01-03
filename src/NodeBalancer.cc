/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * aria2 - The high speed download utility
 *
 * Node Balancer - Multi-IP load balancing for downloads
 */
#include "NodeBalancer.h"

#include <cstring>
#include <sstream>
#include <algorithm>

#include "Logger.h"
#include "LogFactory.h"
#include "fmt.h"
#include "SocketCore.h"
#include "util.h"
#include "uri.h"

namespace aria2 {

std::unique_ptr<NodeBalancer> NodeBalancer::instance_;
std::mutex NodeBalancer::instanceMutex_;

NodeBalancer::NodeBalancer()
    : currentIndex_(0), enabled_(false), nodesFetched_(false)
{
}

NodeBalancer::~NodeBalancer() = default;

NodeBalancer* NodeBalancer::getInstance()
{
  std::lock_guard<std::mutex> lock(instanceMutex_);
  if (!instance_) {
    instance_.reset(new NodeBalancer());
  }
  return instance_.get();
}

void NodeBalancer::init(const std::string& apiUrl,
                        const std::string& targetHost)
{
  std::lock_guard<std::mutex> lock(mutex_);
  apiUrl_ = apiUrl;
  targetHost_ = targetHost;

  if (apiUrl.empty() || targetHost.empty()) {
    enabled_ = false;
    return;
  }

  A2_LOG_INFO(fmt("NodeBalancer initialized: API=%s, Target=%s",
                  apiUrl_.c_str(), targetHost_.c_str()));

  // Note: fetchNodesFromApi() is a blocking call.
  // For HTTPS APIs, this simple implementation won't work.
  // Users should use --node-balancer-ips for manual configuration instead.
  if (apiUrl.find("https://") == 0) {
    A2_LOG_WARN("NodeBalancer: HTTPS API not supported. "
                "Please use --node-balancer-ips for manual configuration.");
    enabled_ = false;
    return;
  }

  // Try to fetch nodes from API
  try {
    fetchNodesFromApi();
  }
  catch (const std::exception& e) {
    A2_LOG_ERROR(fmt("NodeBalancer: Failed to initialize: %s", e.what()));
  }

  enabled_ = !nodeIps_.empty();
}

void NodeBalancer::initWithIps(const std::string& ipList,
                               const std::string& targetHost)
{
  std::lock_guard<std::mutex> lock(mutex_);
  targetHost_ = targetHost;
  nodeIps_.clear();

  // Parse comma-separated IP list
  std::string::size_type start = 0;
  std::string::size_type end;

  while ((end = ipList.find(',', start)) != std::string::npos) {
    std::string ip = ipList.substr(start, end - start);
    // Trim whitespace
    size_t first = ip.find_first_not_of(" \t");
    size_t last = ip.find_last_not_of(" \t");
    if (first != std::string::npos && last != std::string::npos) {
      ip = ip.substr(first, last - first + 1);
      if (!ip.empty()) {
        nodeIps_.push_back(ip);
        A2_LOG_DEBUG(fmt("NodeBalancer: Added manual IP: %s", ip.c_str()));
      }
    }
    start = end + 1;
  }

  // Handle last IP (or only IP if no comma)
  if (start < ipList.size()) {
    std::string ip = ipList.substr(start);
    size_t first = ip.find_first_not_of(" \t");
    size_t last = ip.find_last_not_of(" \t");
    if (first != std::string::npos && last != std::string::npos) {
      ip = ip.substr(first, last - first + 1);
      if (!ip.empty()) {
        nodeIps_.push_back(ip);
        A2_LOG_DEBUG(fmt("NodeBalancer: Added manual IP: %s", ip.c_str()));
      }
    }
  }

  enabled_ = !targetHost.empty() && !nodeIps_.empty();
  nodesFetched_ = enabled_;

  if (enabled_) {
    A2_LOG_INFO(fmt("NodeBalancer initialized with %lu manual IPs for host %s",
                    static_cast<unsigned long>(nodeIps_.size()),
                    targetHost_.c_str()));
  }
}

bool NodeBalancer::isEnabled() const { return enabled_ && !nodeIps_.empty(); }

bool NodeBalancer::shouldBalance(const std::string& host) const
{
  if (!enabled_) {
    return false;
  }
  // Case-insensitive comparison
  return util::strieq(host, targetHost_);
}

std::string NodeBalancer::getNextNodeIp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (nodeIps_.empty()) {
    return "";
  }
  size_t index = currentIndex_.fetch_add(1) % nodeIps_.size();
  return nodeIps_[index];
}

std::string NodeBalancer::getNodeIp(size_t index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= nodeIps_.size()) {
    return "";
  }
  return nodeIps_[index];
}

size_t NodeBalancer::getNodeCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return nodeIps_.size();
}

const std::string& NodeBalancer::getTargetHost() const { return targetHost_; }

void NodeBalancer::addNodeIp(const std::string& ip)
{
  std::lock_guard<std::mutex> lock(mutex_);
  nodeIps_.push_back(ip);
  A2_LOG_DEBUG(fmt("NodeBalancer: Added node IP %s", ip.c_str()));
}

void NodeBalancer::clearNodes()
{
  std::lock_guard<std::mutex> lock(mutex_);
  nodeIps_.clear();
  currentIndex_ = 0;
}

std::vector<std::string> NodeBalancer::getAllNodeIps() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return nodeIps_;
}

void NodeBalancer::markIpFailed(const std::string& ip)
{
  std::lock_guard<std::mutex> lock(mutex_);
  failedIps_[ip]++;
  int failCount = failedIps_[ip];
  A2_LOG_WARN(fmt("NodeBalancer: IP %s failed (count: %d/%d)",
                  ip.c_str(), failCount, MAX_FAILURES));
}

void NodeBalancer::markIpSuccess(const std::string& ip)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = failedIps_.find(ip);
  if (it != failedIps_.end()) {
    failedIps_.erase(it);
    A2_LOG_DEBUG(fmt("NodeBalancer: IP %s recovered", ip.c_str()));
  }
}

std::string NodeBalancer::getWorkingNodeIp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (nodeIps_.empty()) {
    return "";
  }

  // Try to find a working IP (not failed too many times)
  size_t startIndex = currentIndex_++;
  size_t tried = 0;

  while (tried < nodeIps_.size()) {
    size_t index = (startIndex + tried) % nodeIps_.size();
    const std::string& ip = nodeIps_[index];

    auto it = failedIps_.find(ip);
    if (it == failedIps_.end() || it->second < MAX_FAILURES) {
      // This IP is available
      A2_LOG_DEBUG(fmt("NodeBalancer: Selected IP %s (index %zu)",
                       ip.c_str(), index));
      return ip;
    }
    tried++;
  }

  // All IPs have failed too many times, reset and try again
  A2_LOG_WARN("NodeBalancer: All IPs have failed, resetting failure counts");
  failedIps_.clear();

  size_t index = startIndex % nodeIps_.size();
  return nodeIps_[index];
}

void NodeBalancer::resetFailures()
{
  std::lock_guard<std::mutex> lock(mutex_);
  failedIps_.clear();
  A2_LOG_INFO("NodeBalancer: Reset all IP failure counts");
}

size_t NodeBalancer::getAvailableNodeCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t available = 0;
  for (const auto& ip : nodeIps_) {
    auto it = failedIps_.find(ip);
    if (it == failedIps_.end() || it->second < MAX_FAILURES) {
      available++;
    }
  }
  return available;
}

bool NodeBalancer::parseApiResponse(const std::string& jsonResponse)
{
  // Simple JSON parsing for the expected format:
  // {"success":true,"data":{"recommended_nodes":[{"ip":"..."},...]}}
  //
  // We'll do a simple string-based parsing since we don't want to add
  // a JSON library dependency

  nodeIps_.clear();

  // Find "recommended_nodes" array
  size_t pos = jsonResponse.find("\"recommended_nodes\"");
  if (pos == std::string::npos) {
    A2_LOG_ERROR("NodeBalancer: 'recommended_nodes' not found in response");
    return false;
  }

  // Find the array start
  pos = jsonResponse.find('[', pos);
  if (pos == std::string::npos) {
    return false;
  }

  // Find the array end
  size_t endPos = jsonResponse.find(']', pos);
  if (endPos == std::string::npos) {
    return false;
  }

  std::string arrayContent = jsonResponse.substr(pos + 1, endPos - pos - 1);

  // Extract all IP addresses
  size_t searchPos = 0;
  while (true) {
    // Find "ip" key
    size_t ipKeyPos = arrayContent.find("\"ip\"", searchPos);
    if (ipKeyPos == std::string::npos) {
      break;
    }

    // Find the colon after "ip"
    size_t colonPos = arrayContent.find(':', ipKeyPos);
    if (colonPos == std::string::npos) {
      break;
    }

    // Find the opening quote of the IP value
    size_t valueStart = arrayContent.find('"', colonPos);
    if (valueStart == std::string::npos) {
      break;
    }
    valueStart++; // Skip the quote

    // Find the closing quote
    size_t valueEnd = arrayContent.find('"', valueStart);
    if (valueEnd == std::string::npos) {
      break;
    }

    std::string ip = arrayContent.substr(valueStart, valueEnd - valueStart);
    if (!ip.empty()) {
      nodeIps_.push_back(ip);
      A2_LOG_DEBUG(fmt("NodeBalancer: Parsed node IP: %s", ip.c_str()));
    }

    searchPos = valueEnd + 1;
  }

  A2_LOG_INFO(fmt("NodeBalancer: Loaded %lu node IPs",
                  static_cast<unsigned long>(nodeIps_.size())));
  return !nodeIps_.empty();
}

bool NodeBalancer::fetchNodesFromApi()
{
  if (apiUrl_.empty()) {
    return false;
  }

  A2_LOG_INFO(fmt("NodeBalancer: Fetching nodes from %s", apiUrl_.c_str()));

  try {
    // Parse the API URL
    uri_split_result us;
    if (uri_split(&us, apiUrl_.c_str()) != 0) {
      A2_LOG_ERROR(
          fmt("NodeBalancer: Failed to parse API URL: %s", apiUrl_.c_str()));
      return false;
    }

    std::string host = uri::getFieldString(us, USR_HOST, apiUrl_.c_str());
    std::string path = uri::getFieldString(us, USR_PATH, apiUrl_.c_str());
    std::string protocol = uri::getFieldString(us, USR_SCHEME, apiUrl_.c_str());

    if (path.empty()) {
      path = "/";
    }

    uint16_t port = 80;
    if (us.port != 0) {
      port = us.port;
    }
    else if (protocol == "https") {
      port = 443;
    }

    // Create socket and connect
    SocketCore socket;
    socket.establishConnection(host, port);

    // Build HTTP request
    std::string request = "GET " + path + " HTTP/1.1\r\n";
    request += "Host: " + host + "\r\n";
    request += "User-Agent: aria2\r\n";
    request += "Accept: application/json\r\n";
    request += "Connection: close\r\n";
    request += "\r\n";

    // Send request
    socket.writeData(request.c_str(), request.size());

    // Read response
    std::string response;
    char buf[4096];
    int emptyReadCount = 0;
    const int maxEmptyReads = 50;

    while (emptyReadCount < maxEmptyReads) {
      size_t len = sizeof(buf) - 1;
      socket.readData(buf, len);
      if (len > 0) {
        buf[len] = '\0';
        response += buf;
        emptyReadCount = 0;

        // Check if we have complete response
        size_t headerEnd = response.find("\r\n\r\n");
        if (headerEnd != std::string::npos) {
          // Simple check: if we have closing bracket after headers
          if (response.rfind('}') > headerEnd) {
            break;
          }
        }
      }
      else {
        // No data available or connection closed
        emptyReadCount++;
      }
    }

    // Find the body (after \r\n\r\n)
    size_t bodyStart = response.find("\r\n\r\n");
    if (bodyStart == std::string::npos) {
      A2_LOG_ERROR("NodeBalancer: Invalid HTTP response - no body found");
      return false;
    }

    std::string body = response.substr(bodyStart + 4);
    if (body.empty()) {
      A2_LOG_ERROR("NodeBalancer: Empty response body");
      return false;
    }

    // Parse the JSON response
    nodesFetched_ = parseApiResponse(body);
    return nodesFetched_;
  }
  catch (const std::exception& e) {
    A2_LOG_ERROR(fmt("NodeBalancer: Failed to fetch nodes: %s", e.what()));
    return false;
  }
  catch (...) {
    A2_LOG_ERROR("NodeBalancer: Unknown error while fetching nodes");
    return false;
  }
}

bool NodeBalancer::refreshNodes() { return fetchNodesFromApi(); }

} // namespace aria2
