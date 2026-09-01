/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/task/teleop/mppi_assist.hpp"
#include "autonomy/task/teleop/obstacle_grid.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/task/teleop/constants.hpp"

namespace autonomy::task::teleop {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

/**
 * @brief Build default rolling costmap options for teleop
 */
map::proto::Costmap2DOptions DefaultCostmapOptions(
    const TeleopMppiAssist::LaserScanOptions& laser_scan,
    const PointCloudObstacleFeeder::Options& point_cloud,
    bool use_laser_for_costmap) {
    map::proto::Costmap2DOptions options;
    options.set_enabled(true);
    options.set_frame_id(kDefaultBaseFrame);
    options.set_name("teleop_local_costmap");
    options.set_resolution(0.05);
    options.set_width(3);
    options.set_height(3);
    options.set_update_frequency(20.0);
    options.set_rolling_window(true);
    options.set_robot_radius(0.22);
    // Sensor-only: disable static /map layer (wrapper also sets track_unknown=false).
    options.mutable_static_layer()->set_enabled(false);
    options.clear_map_file();

    options.add_plugins("obstacle_layer");
    options.add_plugins("inflation_layer");

    auto* obstacle = options.mutable_obstacle_layer();
    obstacle->set_enabled(true);
    obstacle->set_footprint_clearing_enabled(true);
    auto& sources = *obstacle->mutable_sensor_sources();
    if (use_laser_for_costmap && !laser_scan.topic.empty()) {
        auto& scan = sources["scan"];
        scan.set_topic(laser_scan.topic);
        scan.set_data_type("LaserScan");
        scan.set_marking(true);
        scan.set_clearing(true);
        scan.set_raytrace_max_range(5.0);
    }
    if (!point_cloud.cloud_topic.empty() ||
        !point_cloud.depth_topic.empty()) {
        auto& src = sources["rgbd_cloud"];
        src.set_topic(point_cloud.cloud_topic.empty()
                          ? point_cloud.depth_topic
                          : point_cloud.cloud_topic);
        src.set_data_type("PointCloud2");
        src.set_marking(true);
        src.set_clearing(false);
        src.set_min_obstacle_height(-0.15);
        src.set_max_obstacle_height(1.2);
        src.set_raytrace_max_range(
            point_cloud.max_depth_m > 0.0 ? point_cloud.max_depth_m : 3.5);
    }

    auto* inflation = options.mutable_inflation_layer();
    inflation->set_enabled(true);
    inflation->set_cost_scaling_factor(3.0);
    inflation->set_inflation_radius(0.55);
    return options;
}

/**
 * @brief Build default MPPI controller options for teleop
 */
control::proto::MPPIControllerOptions DefaultMppiOptions() {
    control::proto::MPPIControllerOptions options;
    options.add_critics("CostCritic");
    options.add_critics("PathFollowCritic");
    options.add_critics("PreferForwardCritic");
    options.add_critics("ConstraintCritic");
    options.add_critics("TwirlingCritic");
    options.set_model_dt(0.05);
    options.set_time_steps(56);
    options.set_batch_size(2000);
    options.set_iteration_count(1);
    options.set_temperature(0.3);
    options.set_gamma(0.015);
    options.set_vx_max(0.5);
    options.set_vx_min(-0.35);
    options.set_vy_max(0.5);
    options.set_wz_max(1.9);
    options.mutable_cost_critic()->set_consider_footprint(false);
    return options;
}

/**
 * @brief Identity pose in the given frame
 */
automsgs::msgs::geometry_msgs::PoseStamped IdentityRobotPose(
    const std::string& frame_id) {
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id(frame_id);
    *pose.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    return pose;
}

}  // namespace

/**
 * @brief Public wrapper for default costmap options
 */
map::proto::Costmap2DOptions TeleopMppiAssist::DefaultCostmapOptions(
    const LaserScanOptions& laser_scan,
    const PointCloudObstacleFeeder::Options& point_cloud,
    bool use_laser_for_costmap) {
    return ::autonomy::task::teleop::DefaultCostmapOptions(
        laser_scan, point_cloud, use_laser_for_costmap);
}

/**
 * @brief Public wrapper for default MPPI options
 */
control::proto::MPPIControllerOptions TeleopMppiAssist::DefaultMppiOptions() {
    return ::autonomy::task::teleop::DefaultMppiOptions();
}

bool TeleopMppiAssist::Configure(
    std::shared_ptr<autolink::Node> node,
    std::shared_ptr<transform::Buffer> transform_buffer,
    const Options& options) {
    configured_ = false;
    node_ = std::move(node);
    transform_buffer_ = std::move(transform_buffer);
    options_ = options;

    if (!options_.enabled) {
        configured_ = true;
        return true;
    }
    if (!node_) {
        AERROR << "TeleopMppiAssist: node is null";
        return false;
    }

    map::proto::Costmap2DOptions costmap_opts = options_.costmap;
    const int override_width = options_.costmap.width();
    const int override_height = options_.costmap.height();
    const double override_resolution = options_.costmap.resolution();
    const double override_update_hz = options_.costmap.update_frequency();
    const double override_robot_radius = options_.costmap.robot_radius();
    if (costmap_opts.plugins_size() == 0 && !costmap_opts.has_obstacle_layer()) {
        costmap_opts = DefaultCostmapOptions(options_.laser_scan,
                                             options_.point_cloud,
                                             options_.use_laser_for_costmap);
    }
    if (override_width > 0) {
        costmap_opts.set_width(override_width);
    }
    if (override_height > 0) {
        costmap_opts.set_height(override_height);
    }
    if (override_resolution > 0.0) {
        costmap_opts.set_resolution(override_resolution);
    }
    if (override_update_hz > 0.0) {
        costmap_opts.set_update_frequency(override_update_hz);
    }
    if (override_robot_radius > 0.0) {
        costmap_opts.set_robot_radius(override_robot_radius);
    }
    robot_radius_ = costmap_opts.robot_radius() > 0.0 ? costmap_opts.robot_radius()
                                                        : 0.22;
    if (!costmap_opts.frame_id().empty() &&
        costmap_opts.frame_id() != kDefaultBaseFrame) {
        AWARN << "TeleopMppiAssist: overriding costmap frame "
              << costmap_opts.frame_id() << " -> " << kDefaultBaseFrame;
    }
    costmap_opts.set_frame_id(kDefaultBaseFrame);

    costmap_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
        costmap_opts, "teleop_local_costmap");
    costmap_->setGlobalFrameID(kDefaultBaseFrame);
    costmap_->setRobotBaseFrameID(kDefaultBaseFrame);

    costmap_writer_.reset();
    if (costmap_opts.enabled() && node_) {
        costmap_writer_ =
            node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                kTeleopLocalCostmapTopic);
        if (costmap_writer_) {
            auto writer = costmap_writer_;
            costmap_->SetMapPublishCallback(
                [writer](const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
                    if (writer) {
                        writer->Write(grid);
                    }
                });
            AINFO << "TeleopMppiAssist: local costmap on "
                  << kTeleopLocalCostmapTopic;
        } else {
            AWARN << "TeleopMppiAssist: failed to create writer "
                  << kTeleopLocalCostmapTopic;
        }
    }

    if (costmap_opts.enabled()) {
        costmap_->Start();
    }
    AINFO << "TeleopMppiAssist: rolling costmap " << costmap_opts.width() << "x"
          << costmap_opts.height() << "m RGB-D-only (no static /map), plugins="
          << costmap_opts.plugins_size();

    scan_reader_.reset();
    last_scan_time_ = {};
    if (options_.use_laser_for_costmap &&
        !options_.laser_scan.topic.empty()) {
        TeleopMppiAssist* self = this;
        scan_reader_ =
            node_->CreateReader<automsgs::msgs::sensor_msgs::LaserScan>(
                options_.laser_scan.topic,
                [self](const std::shared_ptr<
                           automsgs::msgs::sensor_msgs::LaserScan>& msg) {
                    if (!msg || !self->costmap_) {
                        return;
                    }
                    self->costmap_->feedLaserScan(*msg);
                    self->last_scan_time_ = std::chrono::steady_clock::now();
                });
        if (!scan_reader_) {
            AWARN << "TeleopMppiAssist: failed to subscribe "
                  << options_.laser_scan.topic;
        } else {
            AINFO << "TeleopMppiAssist: LaserScan on "
                  << options_.laser_scan.topic;
        }
    }

    if (!options_.point_cloud.cloud_topic.empty() ||
        !options_.point_cloud.depth_topic.empty()) {
        feeder_.Configure(node_, costmap_, options_.point_cloud);
        feeder_.Start();
        AINFO << "TeleopMppiAssist: local costmap input RGB-D"
              << (options_.point_cloud.cloud_topic.empty()
                      ? ""
                      : " PointCloud2 " + options_.point_cloud.cloud_topic)
              << (options_.point_cloud.depth_topic.empty()
                      ? ""
                      : " depth " + options_.point_cloud.depth_topic);
    } else {
        feeder_.Stop();
        AWARN << "TeleopMppiAssist: no RGB-D point cloud source configured";
    }

    selector_.set_point_per_path_thre(options_.path_library.point_per_path_thr);
    selector_.set_dir_weight(options_.path_library.dir_weight);
    PathLibraryOptions library_options = options_.path_library;
    if (library_options.sensor_range <= 0.0 &&
        options_.point_cloud.max_depth_m > 0.0) {
        library_options.sensor_range = options_.point_cloud.max_depth_m;
    }
    if (library_options.path_range <= 0.0) {
        library_options.path_range = 3.0 * library_options.segment_length;
    }
    selector_.GenerateLibrary(library_options);
    SetJoyMaxSpeed(options_.path_library.max_linear_speed);

    path_visualizer_.Configure(node_, kDefaultBaseFrame,
                               options_.publish_path_viz);
    if (options_.publish_path_viz) {
        path_visualizer_.SetLibrarySource(&selector_);
        path_visualizer_.SetPreviewCallback([this]() {
            PublishPreview(last_joy_linear_.load(std::memory_order_relaxed),
                               last_joy_angular_.load(std::memory_order_relaxed));
        });
        const double viz_hz = std::max(1.0, options_.path_viz_rate_hz);
        const auto period = std::chrono::milliseconds(
            static_cast<int>(1000.0 / viz_hz));
        path_visualizer_.StartDiscovery(period);
        path_visualizer_.PublishEmpty(kDefaultBaseFrame);
        AINFO << "TeleopMppiAssist: path viz on " << kTeleopPathTopic << " and "
              << kTeleopFreePathMarkersTopic << " (frame=" << kDefaultBaseFrame
              << ", rate=" << viz_hz << " Hz, candidates="
              << selector_.candidates().size() << ")";
    }

    control::proto::ControllerOptions controller_options;
    if (options_.mppi.critics_size() > 0) {
        *controller_options.mutable_mppi_controller_options() = options_.mppi;
    } else {
        *controller_options.mutable_mppi_controller_options() =
            DefaultMppiOptions();
    }

    mppi_ = std::make_unique<
        control::controller::mppi_controller::MPPIController>();
    mppi_->Configure(controller_options, "teleop_mppi", transform_buffer_, costmap_);
    mppi_->Activate();

    goal_checker_.Initialize("teleop_goal_checker", costmap_);
    goal_checker_.SetTolerances(100.0, 100.0, false);

    configured_ = true;
    AINFO << "TeleopMppiAssist configured (local frame=" << kDefaultBaseFrame
          << ")";
    return true;
}

/**
 * @brief Destructor
 */
TeleopMppiAssist::~TeleopMppiAssist() {
    Shutdown();
}

void TeleopMppiAssist::Shutdown() {
    path_visualizer_.StopDiscovery();
    path_visualizer_.SetLibrarySource(nullptr);
    path_visualizer_.SetPreviewCallback(nullptr);
    path_visualizer_.PublishEmpty(kDefaultBaseFrame);
    if (scan_reader_) {
        scan_reader_->Shutdown();
        scan_reader_.reset();
    }
    last_scan_time_ = {};
    feeder_.Stop();
    if (mppi_) {
        mppi_->Deactivate();
        mppi_.reset();
    }
    if (costmap_) {
        costmap_->SetMapPublishCallback(nullptr);
        costmap_->Stop();
        costmap_.reset();
    }
    costmap_writer_.reset();
    configured_ = false;
}

/**
 * @brief Check LaserScan freshness against stale timeout
 */
bool TeleopMppiAssist::IsScanFresh() const {
    if (options_.laser_scan.topic.empty()) {
        return false;
    }
    if (last_scan_time_.time_since_epoch().count() == 0) {
        return false;
    }
    const auto elapsed =
        std::chrono::steady_clock::now() - last_scan_time_;
    return std::chrono::duration<double>(elapsed).count() <=
           options_.laser_scan.stale_timeout_sec;
}

/**
 * @brief True when RGB-D or laser input is fresh
 */
bool TeleopMppiAssist::IsPerceptionOk() const {
    if (options_.use_laser_for_costmap &&
        !options_.laser_scan.topic.empty() && IsScanFresh()) {
        return true;
    }
    if ((!options_.point_cloud.cloud_topic.empty() ||
         !options_.point_cloud.depth_topic.empty()) &&
        feeder_.IsCloudFresh()) {
        return true;
    }
    return false;
}

/**
 * @brief True after first perception message received
 */
bool TeleopMppiAssist::GotPerception() const {
    if (options_.use_laser_for_costmap &&
        !options_.laser_scan.topic.empty() &&
        last_scan_time_.time_since_epoch().count() != 0) {
        return true;
    }
    return feeder_.HasReceivedCloud();
}

/**
 * @brief Update costmap and return obstacle grid adapter
 */
CostmapObstacleGrid TeleopMppiAssist::RefreshGrid() const {
    costmap_->updateMap();
    return CostmapObstacleGrid(*costmap_->getCostmap());
}

/**
 * @brief Select paths and publish visualization topics
 */
void TeleopMppiAssist::PublishPreview(double joy_linear_x,
                                          double joy_angular_z) {
    if (!options_.publish_path_viz || !configured_) {
        return;
    }
    const std::string frame = kDefaultBaseFrame;

    if (!HasForwardStick(joy_linear_x)) {
        path_visualizer_.PublishEmpty(frame);
        return;
    }

    if (!costmap_) {
        path_visualizer_.PublishLibrary(selector_, frame);
        return;
    }

    const auto grid = RefreshGrid();
    automsgs::msgs::geometry_msgs::Twist preview_speed;
    PathSelectionResult selection =
        SelectPaths(joy_linear_x, joy_angular_z, preview_speed);
    IntentPathSelector::FillPreviewScales(
        &selection, JoySpeedNorm(joy_linear_x, joy_angular_z),
        selector_.library_options());

    if (selection.best_path.has_value()) {
        path_visualizer_.PublishSelected(selection, frame);
    }

    const std::optional<int> best_index =
        selection.best_index >= 0 ? std::optional<int>(selection.best_index)
                                  : std::nullopt;
    path_visualizer_.PublishClipped(selector_, grid, frame, best_index,
                                           &selection);
}

/**
 * @brief Clear path visualization and reset cached stick
 */
void TeleopMppiAssist::ClearPathViz() {
    if (!options_.publish_path_viz || !configured_) {
        return;
    }
    last_joy_linear_.store(0.0, std::memory_order_relaxed);
    last_joy_angular_.store(0.0, std::memory_order_relaxed);
    path_visualizer_.PublishEmpty(kDefaultBaseFrame);
}

/**
 * @brief Republish preview for discovery subscribers
 */
void TeleopMppiAssist::RefreshPreview() {
    if (!configured_ || !options_.publish_path_viz) {
        return;
    }
    PublishPreview(last_joy_linear_.load(std::memory_order_relaxed),
                       last_joy_angular_.load(std::memory_order_relaxed));
}

/**
 * @brief Store latest stick for discovery publisher thread
 */
void TeleopMppiAssist::CacheJoy(double joy_linear_x,
                                                double joy_angular_z) {
    last_joy_linear_.store(joy_linear_x, std::memory_order_relaxed);
    last_joy_angular_.store(joy_angular_z, std::memory_order_relaxed);
}

/**
 * @brief Sync joySpeed normalization with TeleopClient limits
 */
void TeleopMppiAssist::SetJoyMaxSpeed(double max_linear_speed) {
    if (max_linear_speed > 0.0) {
        joy_max_linear_speed_.store(max_linear_speed, std::memory_order_relaxed);
    }
}

/**
 * @brief True when forward stick exceeds stopped epsilon
 */
bool TeleopMppiAssist::HasForwardStick(double joy_linear_x) const {
    return std::abs(joy_linear_x) >= options_.stopped_linear_epsilon;
}

double TeleopMppiAssist::JoySpeedNorm(double joy_linear_x,
                                      double joy_angular_z) const {
    if (!HasForwardStick(joy_linear_x)) {
        return 0.0;
    }
    const double configured_max = joy_max_linear_speed_.load(
        std::memory_order_relaxed);
    const double max_linear =
        configured_max > 1e-6
            ? configured_max
            : (options_.path_library.max_linear_speed > 1e-6
                   ? options_.path_library.max_linear_speed
                   : 0.5);
    return std::clamp(std::hypot(joy_linear_x, joy_angular_z) / max_linear,
                       0.0, 1.0);
}

/**
 * @brief Run path library selection for current stick
 */
PathSelectionResult TeleopMppiAssist::SelectPaths(
    double joy_linear_x, double joy_angular_z,
    const automsgs::msgs::geometry_msgs::Twist& robot_speed) {
    const double joy_dir_deg = JoyDirDeg(joy_angular_z, joy_linear_x);
    const double joy_speed_norm = JoySpeedNorm(joy_linear_x, joy_angular_z);
    PathSelectContext context;
    context.robot_vx = robot_speed.linear().x();
    context.robot_wz = robot_speed.angular().z();
    const CostmapObstacleGrid obstacle_grid(*costmap_->getCostmap());
    return selector_.Select(obstacle_grid, joy_dir_deg, joy_speed_norm, context);
}

/**
 * @brief Return identity base_link pose for local planning
 */
bool TeleopMppiAssist::TryGetRobotPose(
    automsgs::msgs::geometry_msgs::PoseStamped* pose) const {
    if (!pose) {
        return false;
    }
    *pose = IdentityRobotPose(kDefaultBaseFrame);
    return true;
}

/**
 * @brief Write raw twist command to output message
 */
void TeleopMppiAssist::EmitPassthrough(
    double linear_x, double angular_z,
    automsgs::msgs::geometry_msgs::TwistStamped* command_out) const {
    command_out->mutable_header()->set_frame_id(kDefaultBaseFrame);
    *command_out->mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    command_out->mutable_twist()->mutable_linear()->set_x(linear_x);
    command_out->mutable_twist()->mutable_angular()->set_z(angular_z);
}

/**
 * @brief Convert stick vector to intent heading (degrees)
 */
double TeleopMppiAssist::JoyDirDeg(double angular_z,
                                             double linear_x) const {
    constexpr double kLinearEps = 1e-3;
    if (std::abs(linear_x) < kLinearEps) {
        return std::clamp(angular_z * options_.angular_to_dir_gain * kRadToDeg,
                          -90.0, 90.0);
    }
    return std::atan2(angular_z * options_.angular_to_dir_gain, linear_x) *
           kRadToDeg;
}

/**
 * @brief Probe ray along joy heading for obstacles
 */
bool TeleopMppiAssist::IsForwardBlocked(
    const PathObstacleGrid& grid, double joy_dir_deg) const {
    const double probe_length =
        options_.blocked_turn_probe_length > 0.0
            ? options_.blocked_turn_probe_length
            : std::max(0.2, options_.path_library.look_ahead_distance);
    return IsRayBlocked(grid, joy_dir_deg, robot_radius_ * 0.5,
                        std::max(probe_length, robot_radius_ + 0.1));
}

/**
 * @brief Decide if blocked in-place turn is required
 */
bool TeleopMppiAssist::NeedsBlockedTurn(
    const PathObstacleGrid& grid, const PathSelectionResult& selection,
    double joy_linear_x, double joy_angular_z) const {
    if (!options_.blocked_turn_enabled || !HasForwardStick(joy_linear_x)) {
        return false;
    }
    if (!selection.best_path.has_value()) {
        return true;
    }
    const double joy_dir_deg = JoyDirDeg(joy_angular_z, joy_linear_x);
    if (IsForwardBlocked(grid, joy_dir_deg)) {
        return true;
    }
    return PathArcLength(*selection.best_path) < robot_radius_ + 0.08;
}

/**
 * @brief Emit zero linear and safe angular velocity
 */
void TeleopMppiAssist::EmitBlockedTurn(
    const PathObstacleGrid& grid, double joy_linear_x, double joy_angular_z,
    automsgs::msgs::geometry_msgs::TwistStamped* command_out) const {
    const auto& lib = options_.path_library;
    const double joy_dir_deg = JoyDirDeg(joy_angular_z, joy_linear_x);
    const double max_w =
        options_.blocked_turn_max_angular > 0.0
            ? options_.blocked_turn_max_angular
            : 0.8;
    const double w_scale =
        std::max(0.15, JoySpeedNorm(joy_linear_x, joy_angular_z)) * max_w *
        options_.in_place_angular_scale;

    double angular_z = 0.0;
    if (std::abs(joy_dir_deg) >= 1.0 || std::abs(joy_angular_z) >= 1e-3) {
        if (lib.check_rot_obstacle) {
            const double path_scale =
                lib.def_path_scale > 0.0 ? lib.def_path_scale : 1.25;
            const RotObstacleAngles rot_angles =
                IntentPathSelector::RotObstacleScan(
                    grid, path_scale, lib.rot_obstacle_vehicle_length,
                    lib.rot_obstacle_vehicle_width);
            const int rot_dir =
                IntentPathSelector::BestRotDir(joy_dir_deg, rot_angles,
                                                       lib);
            if (rot_dir >= 0) {
                angular_z = (IntentPathSelector::RotDirToDeg(rot_dir) >= 0.0
                                 ? 1.0
                                 : -1.0) *
                            w_scale;
            }
        } else {
            const double turn_sign =
                std::abs(joy_dir_deg) >= 1.0
                    ? (joy_dir_deg >= 0.0 ? 1.0 : -1.0)
                    : (joy_angular_z >= 0.0 ? 1.0 : -1.0);
            angular_z = turn_sign * w_scale;
        }
    }

    EmitPassthrough(0.0, angular_z, command_out);
}

/**
 * @brief Main assist control loop for one cycle
 */
bool TeleopMppiAssist::Tick(double joy_linear_x, double joy_angular_z,
                            const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose,
                            const automsgs::msgs::geometry_msgs::Twist& robot_speed,
                            automsgs::msgs::geometry_msgs::TwistStamped* command_out) {
    if (!command_out || !configured_) {
        return false;
    }

    // Assist disabled: forward joystick command unchanged.
    if (!options_.enabled) {
        EmitPassthrough(joy_linear_x, joy_angular_z, command_out);
        return true;
    }

    PathSelectionResult path_selection;
    if (costmap_ && HasForwardStick(joy_linear_x)) {
        path_selection = SelectPaths(joy_linear_x, joy_angular_z, robot_speed);
    }

    // No fresh perception: passthrough (assist cannot validate obstacles).
    if (!IsPerceptionOk()) {
        EmitPassthrough(joy_linear_x, joy_angular_z, command_out);
        if (GotPerception()) {
            AWARN_EVERY(50) << "TeleopMppiAssist: perception stale, passthrough";
        }
        return true;
    }
    if (!mppi_ || !costmap_) {
        EmitPassthrough(joy_linear_x, joy_angular_z, command_out);
        return true;
    }

    // Angular-only stick: in-place rotation, no path library / MPPI.
    if (!HasForwardStick(joy_linear_x)) {
        EmitPassthrough(0.0,
                           joy_angular_z * options_.in_place_angular_scale,
                           command_out);
        return true;
    }

    const auto grid = RefreshGrid();

    // Forward blocked: zero linear, safe turn toward joystick heading.
    if (NeedsBlockedTurn(grid, path_selection, joy_linear_x, joy_angular_z)) {
        EmitBlockedTurn(grid, joy_linear_x, joy_angular_z, command_out);
        return true;
    }
    if (!path_selection.best_path.has_value()) {
        EmitPassthrough(joy_linear_x, joy_angular_z, command_out);
        return true;
    }

    // Follow selected library path with MPPI.
    auto path = *path_selection.best_path;
    path.mutable_header()->set_frame_id(kDefaultBaseFrame);
    for (auto& pose_stamped : *path.mutable_poses()) {
        pose_stamped.mutable_header()->set_frame_id(kDefaultBaseFrame);
    }

    mppi_->SetPlan(path);

    automsgs::msgs::geometry_msgs::PoseStamped pose = robot_pose;
    if (pose.header().frame_id().empty() ||
        pose.header().frame_id() != kDefaultBaseFrame) {
        pose = IdentityRobotPose(kDefaultBaseFrame);
    }

    automsgs::msgs::geometry_msgs::TwistStamped velocity;
    *velocity.mutable_twist() = robot_speed;

    std::string message;
    automsgs::msgs::geometry_msgs::TwistStamped command;
    const uint32_t result = mppi_->ComputeVelocityCommands(
        pose, velocity, command, &goal_checker_, message);
    if (result != 0) {
        // MPPI infeasible: fall back to blocked-turn instead of open-loop forward.
        AWARN << "TeleopMppiAssist: MPPI failed: " << message
              << " — blocked turn fallback";
        EmitBlockedTurn(grid, joy_linear_x, joy_angular_z, command_out);
        return true;
    }

    *command_out = command;
    if (command_out->header().frame_id().empty()) {
        command_out->mutable_header()->set_frame_id(kDefaultBaseFrame);
    }
    return true;
}

}  // namespace autonomy::task::teleop
