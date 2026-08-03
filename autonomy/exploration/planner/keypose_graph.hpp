/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/exploration/planning_env.hpp"
#include "autonomy/exploration/proto/exploration_options.pb.h"

namespace autonomy {
namespace exploration {

/**
 * @file keypose_graph.hpp
 * @brief Keypose graph for global exploration connectivity.
 */

/**
 * @brief Node in the keypose graph.
 */
struct KeyposeNode {
    int id{-1};     //!< @brief node id
    double x{0.0};  //!< @brief position x [m]
    double y{0.0};  //!< @brief position y [m]
    double z{0.0};  //!< @brief position z [m]
};

/**
 * @class KeyposeGraph
 * @brief Sparse roadmap of keyposes used for global path-cost estimates.
 */
class KeyposeGraph
{
public:
    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options
     */
    explicit KeyposeGraph(const proto::ExplorationOptions& options);

    /**
     * @brief Update parameters from exploration options.
     * @param options Exploration options
     */
    void SetOptions(const proto::ExplorationOptions& options);

    /**
     * @brief Insert / update the robot keypose and connect neighbors.
     * @param x Robot x [m]
     * @param y Robot y [m]
     * @param z Robot z [m]
     * @param env Planning environment for collision / LOS checks
     */
    void UpdateRobotPose(double x, double y, double z, const PlanningEnv& env);

    /**
     * @brief Add a bidirectional edge if not already present.
     * @param from Source node id
     * @param to Destination node id
     * @param cost Edge cost
     */
    void AddEdge(int from, int to, double cost);

    /**
     * @brief Shortest-path cost between two nodes (Dijkstra).
     * @param from Source node id
     * @param to Destination node id
     * @return Path cost, or Euclidean fallback
     */
    double ShortestPathCost(int from, int to) const;

    /**
     * @brief Shortest-path cost from the last robot node to a query point.
     * @param x Query x [m]
     * @param y Query y [m]
     * @param z Query z [m]
     * @return Path cost via the nearest graph node
     */
    double ShortestPathCostToPoint(double x, double y, double z) const;

    /**
     * @brief Find the nearest graph node to a query point.
     * @param x Query x [m]
     * @param y Query y [m]
     * @param z Query z [m]
     * @return Node id, or -1 if empty
     */
    int NearestNode(double x, double y, double z) const;

    /**
     * @brief Dijkstra node-id sequence between two graph nodes.
     * @param from Source node id
     * @param to Destination node id
     * @return Node ids along the path (empty if disconnected)
     */
    std::vector<int> ShortestPathNodeIds(int from, int to) const;

    /**
     * @brief Geometric shortest path via the keypose graph (fallback Euclidean).
     * @param from_x Start x [m]
     * @param from_y Start y [m]
     * @param from_z Start z [m]
     * @param to_x Goal x [m]
     * @param to_y Goal y [m]
     * @param to_z Goal z [m]
     * @return Path points including start / goal
     */
    std::vector<automsgs::msgs::geometry_msgs::Point> ShortestPathPoints(
        double from_x, double from_y, double from_z, double to_x, double to_y,
        double to_z) const;

    /**
     * @brief Get all keypose nodes.
     * @return Node list
     */
    const std::vector<KeyposeNode>& nodes() const { return nodes_; }

private:
    /**
     * @brief Insert a node or reuse a nearby existing one.
     * @param x Position x [m]
     * @param y Position y [m]
     * @param z Position z [m]
     * @return Node id
     */
    int AddOrGetNode(double x, double y, double z);

    /**
     * @brief Connect a node to nearby, LOS-clear neighbors.
     * @param node_id Node to connect
     * @param env Planning environment
     */
    void ConnectToNeighbors(int node_id, const PlanningEnv& env);

    /**
     * @brief Dijkstra distances and parents from a source node.
     * @param from Source node id
     * @param dist Output distances
     * @param parent Output parents (-1 for root / unreachable)
     * @return true if from is valid
     */
    bool Dijkstra(int from, std::vector<double>* dist,
                  std::vector<int>* parent) const;

    /**
     * @brief Build a Point from node coordinates.
     * @param node Graph node
     * @return Point message
     */
    static automsgs::msgs::geometry_msgs::Point ToPoint(const KeyposeNode& node);

    proto::ExplorationOptions options_;  //!< @brief exploration options
    std::vector<KeyposeNode> nodes_;     //!< @brief graph nodes
    std::vector<std::vector<std::pair<int, double>>>
        adj_;                  //!< @brief adjacency lists (id, cost)
    int last_robot_node_{-1};  //!< @brief latest robot node id
};

}  // namespace exploration
}  // namespace autonomy
