/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * aria2 - The high speed download utility
 *
 * Node Balancer - Multi-IP load balancing for downloads
 * Fetches recommended node IPs from API and distributes connections
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef D_NODE_BALANCER_H
#define D_NODE_BALANCER_H

#include "common.h"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace aria2 {

class NodeBalancer {
private:
  // Singleton instance
  static std::unique_ptr<NodeBalancer> instance_;
  static std::mutex instanceMutex_;

  // Node API URL (e.g., "https://hs.cs16.fullcone.cn/realip")
  std::string apiUrl_;

  // Target host to replace (e.g., "pan.cdn.fullcone.cn")
  std::string targetHost_;

  // List of recommended node IPs
  std::vector<std::string> nodeIps_;

  // Current index for round-robin selection
  std::atomic<size_t> currentIndex_;

  // Mutex for thread-safe operations
  mutable std::mutex mutex_;

  // Whether the balancer is enabled
  bool enabled_;

  // Whether nodes have been fetched
  bool nodesFetched_;

  // Track failed IPs with failure count
  std::map<std::string, int> failedIps_;

  // Maximum failures before marking IP as bad
  static const int MAX_FAILURES = 3;

  // Time to wait before retrying a failed IP (in seconds)
  static const int RETRY_INTERVAL = 300;

  // Private constructor for singleton
  NodeBalancer();

  // Parse JSON response from API
  bool parseApiResponse(const std::string& jsonResponse);

  // Fetch nodes from API (HTTP request)
  bool fetchNodesFromApi();

public:
  ~NodeBalancer();

  // Get singleton instance
  static NodeBalancer* getInstance();

  // Initialize with API URL and target host
  void init(const std::string& apiUrl, const std::string& targetHost);

  // Initialize with manual IP list and target host
  void initWithIps(const std::string& ipList, const std::string& targetHost);

  // Check if balancer is enabled
  bool isEnabled() const;

  // Check if the given host should be balanced
  bool shouldBalance(const std::string& host) const;

  // Get next node IP (round-robin)
  std::string getNextNodeIp();

  // Get specific node IP by index
  std::string getNodeIp(size_t index) const;

  // Get total number of available nodes
  size_t getNodeCount() const;

  // Get the original target host (for Host header)
  const std::string& getTargetHost() const;

  // Manually add node IPs (for testing or manual configuration)
  void addNodeIp(const std::string& ip);

  // Clear all nodes
  void clearNodes();

  // Refresh nodes from API
  bool refreshNodes();

  // Get all node IPs
  std::vector<std::string> getAllNodeIps() const;

  // Mark an IP as failed (called when connection fails)
  void markIpFailed(const std::string& ip);

  // Mark an IP as successful (called when connection succeeds)
  void markIpSuccess(const std::string& ip);

  // Get a working IP, skipping failed ones
  std::string getWorkingNodeIp();

  // Reset all failure counts
  void resetFailures();

  // Get number of available (non-failed) IPs
  size_t getAvailableNodeCount() const;
};

} // namespace aria2

#endif // D_NODE_BALANCER_H
