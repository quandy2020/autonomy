/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <cstdint>
#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/commsgs/proto/visualization_msgs.pb.h"
#include "autonomy/commsgs/strata_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/map/strata/map_features/feature_manager.hpp"
#include "autonomy/map/strata/navigation/pathfinder.hpp"
#include "autonomy/map/strata/robot/robot_engine.hpp"
#include "autonomy/map/strata/strata_engine.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace bridge {

using MarkerArrayProto = commsgs::proto::visualization_msgs::MarkerArray;

struct StrataBridgeOptions {
    std::string channel_prefix{"/strata"};
    std::string frame_id{"map"};
    double publish_rate_hz{10.0};
    MapViewOptions map_view{};
    SlamMapOptions slam_map{};
    bool load_slam_map{false};
    std::vector<FloorConfig> floors;
    std::string default_floor_id;

    /** 启用 RobotEngine + Pathfinder 仿真（tick 驱动 DisplayManager / FOV）。 */
    bool enable_robot_sim{false};
    double map_width_meters{10.};
    double map_height_meters{10.};
    std::string demo_robot_id{"demo-robot"};
    std::string demo_robot_name{"Demo Robot"};
    double robot_start_x{1.};
    double robot_start_y{1.};
    double robot_goal_x{9.};
    double robot_goal_y{9.};
    bool auto_move_on_start{false};
    double robot_tick_hz{20.};
    bool seed_demo_forbidden_zone{true};
};

class StrataBridgeNode
{
public:
    explicit StrataBridgeNode(StrataBridgeOptions options);

    bool Init(const std::shared_ptr<autolink::Node>& node);
    void SpinUntilShutdown();

private:
    std::string Channel(const std::string& suffix) const;
    void PublishScene(uint64_t stamp_ns);
    void OnFloorSwitch(const std::string& floor_id);
    void OnRobotGoal(double x, double y);
    bool InitRobotSimulation();
    void RefreshPathfinderObstacles();
    void TickRobots();
    uint64_t NowNs() const;

    StrataBridgeOptions options_;
    StrataEngine engine_;
    map_features::FloorManager::SharedPtr floor_manager_;
    map_features::SemanticZones::SharedPtr semantic_zones_;
    robot::RobotEngine::SharedPtr robot_engine_;
    navigation::Pathfinder::SharedPtr pathfinder_;
    double map_width_meters_{0.};
    double map_height_meters_{0.};
    std::vector<LngLat> planned_path_;
    std::shared_ptr<autolink::Node> node_;
    int64_t last_revision_{-1};

    std::shared_ptr<autolink::Writer<commsgs::map_msgs::OccupancyGrid>> map_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::PoiMarkerArray>> poi_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::RobotMarkerArray>> robot_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::SemanticZoneArray>> zone_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::RoadGraph>> road_graph_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::FloorInfoArray>> floors_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::CanvasLabelArray>> canvas_labels_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::LabelBubbleArray>> label_bubbles_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::IotBubbleArray>> iot_bubbles_writer_;
    std::shared_ptr<autolink::Writer<commsgs::strata_msgs::Robot3DLayerArray>> robot_3d_writer_;
    std::shared_ptr<autolink::Writer<MarkerArrayProto>> marker_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>> path_writer_;
    std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> floor_switch_reader_;
    std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> robot_goal_reader_;
};

}  // namespace bridge
}  // namespace strata
}  // namespace map
}  // namespace autonomy
