/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace integration {

enum class GraphVertexKind { kNode, kChannel, kService };

enum class GraphEdgeKind {
  kPublish,
  kSubscribe,
  kRelay,
  kServiceServer,
  kServiceClient,
};

struct GraphVertex {
  std::string id;
  GraphVertexKind kind = GraphVertexKind::kNode;
  std::string label;
  std::string detail;
};

struct GraphEdge {
  std::string from_id;
  std::string to_id;
  GraphEdgeKind kind = GraphEdgeKind::kPublish;
};

struct TopologyGraphBuildOptions {
  bool show_services = true;
  bool show_channels = true;
  std::string filter_text;
  /** Empty means all prefixes; otherwise only channels under this prefix. */
  std::string prefix_filter;
};

struct TopologyGraph {
  std::vector<GraphVertex> vertices;
  std::vector<GraphEdge> edges;
  /** Stable fingerprint used to skip expensive scene rebuilds. */
  std::string topology_hash;
};

/** Builds a channel/node/service graph from Autolink topology discovery. */
TopologyGraph BuildTopologyGraph(const TopologyGraphBuildOptions& options);

/** Lists first-path-segment prefixes for channel grouping filters. */
std::vector<std::string> ListChannelPrefixGroups();

}  // namespace integration
}  // namespace autoviz
