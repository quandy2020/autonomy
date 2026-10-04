/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file topology_graph_builder.hpp
 * @brief Builds a node / channel / service graph from Autolink topology.
 *
 * Consumed by the Channel Graph panel: vertices and edges plus a stable
 * @c topology_hash so the scene can skip expensive rebuilds when unchanged.
 *
 * @see TopologyGraph
 * @see TopologyGraphBuildOptions
 * @see ChannelManager
 */

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace integration {

/**
 * @enum GraphVertexKind
 * @brief Discriminator for a @ref GraphVertex in the topology graph.
 */
enum class GraphVertexKind {
  kNode,     /**< Autolink process / node participant. */
  kChannel,  /**< Pub/sub channel. */
  kService,  /**< Service name (request/response). */
};

/**
 * @enum GraphEdgeKind
 * @brief Relationship drawn between two @ref GraphVertex ids.
 */
enum class GraphEdgeKind {
  kPublish,        /**< Node → channel (writer). */
  kSubscribe,      /**< Channel → node (reader), or node ← channel depending on layout. */
  kRelay,          /**< Intermediate relay hop. */
  kServiceServer,  /**< Node provides a service. */
  kServiceClient,  /**< Node calls a service. */
};

/**
 * @struct GraphVertex
 * @brief One node in the topology graph (node, channel, or service).
 */
struct GraphVertex {
  /** Stable unique id used by edges (@c from_id / @c to_id). */
  std::string id;
  /** Vertex category. */
  GraphVertexKind kind = GraphVertexKind::kNode;
  /** Short display label. */
  std::string label;
  /** Optional secondary text (type, host, …). */
  std::string detail;
};

/**
 * @struct GraphEdge
 * @brief Directed link between two vertex ids.
 */
struct GraphEdge {
  /** Source vertex @ref GraphVertex::id. */
  std::string from_id;
  /** Destination vertex @ref GraphVertex::id. */
  std::string to_id;
  /** Edge semantic. */
  GraphEdgeKind kind = GraphEdgeKind::kPublish;
};

/**
 * @struct TopologyGraphBuildOptions
 * @brief Filters applied when constructing a @ref TopologyGraph.
 */
struct TopologyGraphBuildOptions {
  /** Include service vertices and service edges when @c true. */
  bool show_services = true;
  /** Include channel vertices and pub/sub edges when @c true. */
  bool show_channels = true;
  /**
   * @brief Hide high-noise channels (TF / parameter / rosout), rqt Quiet-like.
   */
  bool quiet_mode = false;
  /**
   * @brief Hide channels with fewer than two unique endpoints (leaf topics).
   */
  bool hide_leaf_channels = false;
  /**
   * @brief Hide channels that have writers but no readers (dead ends).
   */
  bool hide_dead_end_channels = false;
  /** Case-insensitive substring filter on labels / names (empty = no filter).
   *  Comma-separated terms; leading @c - excludes (e.g. @c "sensing,-/tf"). */
  std::string filter_text;
  /**
   * @brief Channel path-prefix filter.
   *
   * Empty means all prefixes; otherwise only channels under this prefix
   * (first path segment grouping — see @ref ListChannelPrefixGroups).
   */
  std::string prefix_filter;
};

/**
 * @struct TopologyGraph
 * @brief Immutable-ish snapshot of topology vertices, edges, and hash.
 */
struct TopologyGraph {
  /** Graph nodes. */
  std::vector<GraphVertex> vertices;
  /** Graph links. */
  std::vector<GraphEdge> edges;
  /**
   * @brief Stable fingerprint of the topology content.
   *
   * Used by the Channel Graph view to skip expensive scene rebuilds when the
   * discovered topology has not changed.
   */
  std::string topology_hash;
};

/**
 * @brief Builds a channel/node/service graph from Autolink topology discovery.
 *
 * @param options Include flags and text / prefix filters.
 * @return Populated @ref TopologyGraph (may be empty if topology is unavailable).
 *
 * @see ListChannelPrefixGroups
 */
TopologyGraph BuildTopologyGraph(const TopologyGraphBuildOptions& options);

/**
 * @brief Lists first-path-segment prefixes for channel grouping filters.
 *
 * Example: channels @c /sensing/lidar and @c /sensing/camera yield prefix
 * @c sensing (exact grouping rules follow the implementation).
 *
 * @return Sorted unique prefix strings for the UI filter combo.
 */
std::vector<std::string> ListChannelPrefixGroups();

}  // namespace integration
}  // namespace autoviz
