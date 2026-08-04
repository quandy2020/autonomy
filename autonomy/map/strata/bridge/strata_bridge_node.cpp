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

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/std_msgs.pb.h"
#include "autolink/time/time.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/map/strata/bridge/pathfinder_utils.hpp"
#include "autonomy/map/strata/bridge/scene_converter.hpp"
#include "autonomy/map/strata/bridge/strata_bridge_node.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/robot/core/robot_profile.hpp"
#include "autonomy/map/strata/robot/visual/robot3d_presets.hpp"

#include <algorithm>

namespace autonomy {
namespace map {
namespace strata {
namespace bridge {

namespace {

std::pair<double, double> ResolveMapSize(const StrataBridgeOptions& options) {
    const auto slam_size = MapSizeFromSlam(options.slam_map);
    if (slam_size.first > 0. && slam_size.second > 0.) {
        return slam_size;
    }
    return {options.map_width_meters, options.map_height_meters};
}

void SeedDemoForbiddenZone(map_features::SemanticZones::SharedPtr zones, double width,
                           double height) {
    if (!zones || width <= 0. || height <= 0.) {
        return;
    }
    const double cx = width * 0.5;
    const double band = std::max(0.5, width * 0.08);
    zones->AddZone("demo-forbidden", ZoneType::kForbidden,
                   {{cx - band, 0., 0.},
                    {cx + band, 0., 0.},
                    {cx + band, height, 0.},
                    {cx - band, height, 0.},
                    {cx - band, 0., 0.}});
}

}  // namespace

StrataBridgeNode::StrataBridgeNode(StrataBridgeOptions options)
    : options_(std::move(options)) {}

bool StrataBridgeNode::Init(const std::shared_ptr<autolink::Node>& node) {
    node_ = node;
    engine_.Init({});
    engine_.CreateMap(options_.map_view);
    if (options_.load_slam_map) {
        engine_.LoadSlamMap(options_.slam_map);
    }

    const auto map_size = ResolveMapSize(options_);
    map_width_meters_ = map_size.first;
    map_height_meters_ = map_size.second;

    if (!options_.floors.empty()) {
        floor_manager_ = engine_.CreateFloorManager(options_.floors, options_.default_floor_id);
        const std::string floor_id =
            options_.default_floor_id.empty() ? options_.floors.front().id
                                              : options_.default_floor_id;
        floor_manager_->BindLayers3D(
            floor_id, {LayerIds::kRobotFov, LayerIds::kRobot3D, LayerIds::kBuilding3D});
    }

    if (options_.enable_robot_sim && !InitRobotSimulation()) {
        AERROR << "Failed to initialize robot simulation.";
        return false;
    }

    map_writer_ = node_->CreateWriter<commsgs::map_msgs::OccupancyGrid>(Channel("map"));
    poi_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::PoiMarkerArray>(Channel("poi_markers"));
    robot_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::RobotMarkerArray>(Channel("robot_markers"));
    zone_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::SemanticZoneArray>(Channel("semantic_zones"));
    road_graph_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::RoadGraph>(Channel("road_graph"));
    floors_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::FloorInfoArray>(Channel("floors"));
    canvas_labels_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::CanvasLabelArray>(Channel("canvas_labels"));
    label_bubbles_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::LabelBubbleArray>(Channel("label_bubbles"));
    iot_bubbles_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::IotBubbleArray>(Channel("iot_bubbles"));
    robot_3d_writer_ =
        node_->CreateWriter<commsgs::strata_msgs::Robot3DLayerArray>(Channel("robot_3d_layers"));
    marker_writer_ = node_->CreateWriter<MarkerArrayProto>(Channel("markers"));
    path_writer_ = node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(Channel("path"));

    floor_switch_reader_ = node_->CreateReader<autolink::message::RawMessage>(
        Channel("floor_switch"),
        [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
            if (msg == nullptr) {
                return;
            }
            commsgs::proto::std_msgs::String proto;
            if (proto.ParseFromString(msg->message)) {
                OnFloorSwitch(proto.data());
            }
        });

    if (options_.enable_robot_sim) {
        robot_goal_reader_ = node_->CreateReader<autolink::message::RawMessage>(
            Channel("robot_goal"),
            [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
                if (msg == nullptr) {
                    return;
                }
                commsgs::proto::geometry_msgs::Point proto;
                if (proto.ParseFromString(msg->message)) {
                    OnRobotGoal(proto.x(), proto.y());
                }
            });
    }

    PublishScene(NowNs());
    return map_writer_ != nullptr && marker_writer_ != nullptr;
}

bool StrataBridgeNode::InitRobotSimulation() {
    if (map_width_meters_ <= 0. || map_height_meters_ <= 0.) {
        AERROR << "Robot simulation requires positive map_width_meters/map_height_meters.";
        return false;
    }

    semantic_zones_ = engine_.CreateSemanticZones();
    if (options_.seed_demo_forbidden_zone && semantic_zones_->GetZones().empty()) {
        SeedDemoForbiddenZone(semantic_zones_, map_width_meters_, map_height_meters_);
    }

    navigation::PathfinderOptions pf_options;
    pf_options.widthMeters = map_width_meters_;
    pf_options.heightMeters = map_height_meters_;
    pathfinder_ = engine_.CreatePathfinder(pf_options);

    robot::RobotEngineConfig robot_config;
    robot_config.autoSyncDisplay = true;
    robot_engine_ = engine_.CreateRobotEngine(robot_config);
    robot_engine_->SetScene(engine_.GetSceneState());
    robot_engine_->SetPathfinder(pathfinder_);

    const auto profile =
        engine_.CreateRobotProfile(robot::core::RobotProfileTypes::kPatrolOutdoor);

    auto status_renderer = engine_.CreateStatusRobotMarkers();
    robot_engine_->GetDisplayManager()->SetRenderer2d(status_renderer);

    const auto model_config = robot::visual::ModelConfigFromProfile(
        profile, engine_.GetResourcePaths());
    auto renderer_3d = engine_.CreateRobot3DStatusLayer(model_config);
    robot_engine_->GetDisplayManager()->SetRenderer3d(renderer_3d);

    RefreshPathfinderObstacles();

    robot::RobotInitialState initial;
    initial.position.x = options_.robot_start_x;
    initial.position.y = options_.robot_start_y;
    initial.battery = 100.f;
    robot_engine_->AddRobot(options_.demo_robot_id, profile, initial);
    robot_engine_->CreateFovForRobot(options_.demo_robot_id);

    RobotMarker marker;
    marker.id = options_.demo_robot_id;
    marker.name = options_.demo_robot_name;
    marker.lngLat = initial.position;
    marker.visible = true;
    robot_engine_->GetDisplayManager()->AddRobot(marker);

    robot_engine_->Start();

    if (options_.auto_move_on_start) {
        OnRobotGoal(options_.robot_goal_x, options_.robot_goal_y);
    }
    return true;
}

void StrataBridgeNode::RefreshPathfinderObstacles() {
    if (!robot_engine_ || !pathfinder_) {
        return;
    }
    const auto& zones = engine_.GetSceneState()->semanticZones;
    robot_engine_->SetPathfinderObstacles(
        BuildPathfinderObstacles(zones, map_width_meters_, map_height_meters_));
}

void StrataBridgeNode::OnRobotGoal(double x, double y) {
    if (!robot_engine_) {
        return;
    }
    LngLat target;
    target.x = x;
    target.y = y;
    target.z = 0.;

    planned_path_.clear();
    if (auto robot = robot_engine_->GetRobot(options_.demo_robot_id)) {
        robot_engine_->PlanPath(robot->state().position, target, planned_path_);
    }
    if (planned_path_.empty()) {
        planned_path_.push_back(target);
    }

    robot_engine_->MoveRobotTo(options_.demo_robot_id, target);
    engine_.GetSceneState()->Touch();
    PublishScene(NowNs());
}

void StrataBridgeNode::TickRobots() {
    if (!robot_engine_ || options_.robot_tick_hz <= 0.) {
        return;
    }
    const double tick_ms = 1000.0 / options_.robot_tick_hz;
    robot_engine_->Tick(tick_ms);
}

void StrataBridgeNode::SpinUntilShutdown() {
    autolink::Rate rate(options_.publish_rate_hz);
    while (autolink::OK()) {
        if (robot_engine_) {
            TickRobots();
        }
        const int64_t revision = engine_.GetSceneState()->revision();
        if (revision != last_revision_) {
            PublishScene(NowNs());
            last_revision_ = revision;
        }
        rate.Sleep();
    }
}

std::string StrataBridgeNode::Channel(const std::string& suffix) const {
    if (options_.channel_prefix.empty()) {
        return "/" + suffix;
    }
    if (options_.channel_prefix.back() == '/') {
        return options_.channel_prefix + suffix;
    }
    return options_.channel_prefix + "/" + suffix;
}

void StrataBridgeNode::PublishScene(uint64_t stamp_ns) {
    const auto exported = engine_.ExportScene();
    const auto messages = ConvertExportedScene(exported, options_.frame_id, stamp_ns);

    if (map_writer_ && exported.occupancyGrid != nullptr) {
        map_writer_->Write(*exported.occupancyGrid);
    }
    if (poi_writer_) {
        poi_writer_->Write(messages.poi_markers);
    }
    if (robot_writer_) {
        robot_writer_->Write(messages.robot_markers);
    }
    if (zone_writer_) {
        zone_writer_->Write(messages.semantic_zones);
    }
    if (road_graph_writer_ && !messages.road_graph.nodes.empty()) {
        road_graph_writer_->Write(messages.road_graph);
    }
    if (floors_writer_ && !messages.floors.floors.empty()) {
        floors_writer_->Write(messages.floors);
    }
    if (canvas_labels_writer_ && !messages.canvas_labels.labels.empty()) {
        canvas_labels_writer_->Write(messages.canvas_labels);
    }
    if (label_bubbles_writer_ && !messages.label_bubbles.bubbles.empty()) {
        label_bubbles_writer_->Write(messages.label_bubbles);
    }
    if (iot_bubbles_writer_ && !messages.iot_bubbles.bubbles.empty()) {
        iot_bubbles_writer_->Write(messages.iot_bubbles);
    }
    if (robot_3d_writer_ && !messages.robot_3d_layers.layers.empty()) {
        robot_3d_writer_->Write(messages.robot_3d_layers);
    }
    if (marker_writer_) {
        marker_writer_->Write(messages.markers);
    }
    if (path_writer_) {
        if (!planned_path_.empty()) {
            path_writer_->Write(BuildPathMessage(planned_path_, messages.path.header));
        } else if (!messages.path.poses.empty()) {
            path_writer_->Write(messages.path);
        }
    }
    engine_.SyncRender();
}

void StrataBridgeNode::OnFloorSwitch(const std::string& floor_id) {
    if (!floor_manager_) {
        AWARN << "Floor switch ignored: floor manager not configured.";
        return;
    }
    if (!floor_manager_->SwitchTo(floor_id)) {
        AWARN << "Floor switch failed for id: " << floor_id;
        return;
    }
    last_revision_ = engine_.GetSceneState()->revision();
    PublishScene(NowNs());
}

uint64_t StrataBridgeNode::NowNs() const {
    const auto now = autolink::Time::Now();
    return static_cast<uint64_t>(now.ToNanosecond());
}

}  // namespace bridge
}  // namespace strata
}  // namespace map
}  // namespace autonomy
