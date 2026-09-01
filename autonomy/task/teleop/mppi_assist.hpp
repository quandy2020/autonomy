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

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/task/teleop/path_selector.hpp"
#include "autonomy/task/teleop/cloud_feeder.hpp"
#include "autonomy/task/teleop/path_viz.hpp"
#include "autonomy/transform/buffer.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>

namespace autonomy::task::teleop {

/**
 * @class teleop::TeleopMppiAssist
 * @brief Smart teleop assist: joystick intent → CMU path library → MPPI → safe cmd_vel
 *
 * All planning and visualization run in base_link. A rolling local costmap is
 * built from RGB-D (and optionally LaserScan). When forward motion is blocked,
 * linear velocity is zeroed and a collision-aware in-place turn is commanded.
 */
class TeleopMppiAssist
{
public:
    // LaserScan subscription options for the local costmap obstacle layer.
    struct LaserScanOptions {
        // Topic name for LaserScan input.
        std::string topic;
        // Max age before scan data is considered stale (seconds).
        double stale_timeout_sec{0.5};
    };

    // Runtime configuration loaded from teleop_assist.lua.
    struct Options {
        // Master enable for assist pipeline.
        bool enabled{true};
        // Publish path and free_path_markers preview topics.
        bool publish_path_viz{true};
        // When false, local costmap uses RGB-D point cloud only.
        bool use_laser_for_costmap{false};
        // free_path_markers + path preview publish rate (Hz).
        double path_viz_rate_hz{15.0};
        // Scales angular stick into heading offset (deg) for path selection.
        double angular_to_dir_gain{1.0};
        // |linear_x| below this is treated as no forward stick (CMU joySpeed).
        double stopped_linear_epsilon{0.02};
        // Scale factor for in-place / blocked-turn angular velocity.
        double in_place_angular_scale{0.5};
        // When forward motion is blocked, command in-place turn (linear_x=0).
        bool blocked_turn_enabled{true};
        // Ray length (m) along joy direction to detect forward blockage.
        double blocked_turn_probe_length{0.35};
        // Max |angular_z| (rad/s) during blocked turn.
        double blocked_turn_max_angular{0.8};
        // CMU/FALCO path library and selection parameters.
        PathLibraryOptions path_library;
        // LaserScan source for optional costmap layer.
        LaserScanOptions laser_scan;
        // RGB-D point cloud feeder options.
        PointCloudObstacleFeeder::Options point_cloud;
        // Rolling local costmap overrides.
        map::proto::Costmap2DOptions costmap;
        // MPPI controller tuning for path following.
        control::proto::MPPIControllerOptions mppi;
    };

    /**
     * @brief Configure costmap, path library, MPPI, and visualization publishers
     * @param node Autolink node for readers/writers
     * @param transform_buffer TF buffer passed to MPPI controller
     * @param options Assist configuration
     * @return false if a required component failed to initialize
     */
    bool Configure(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<transform::Buffer> transform_buffer,
                   const Options& options);

    /**
     * @brief Destructor; calls Shutdown()
     */
    ~TeleopMppiAssist();

    /**
     * @brief Stop publishers, costmap, and MPPI; release resources
     */
    void Shutdown();

    /**
     * @brief Compute one control cycle from joystick input
     * @param joy_linear_x Commanded linear.x from teleop stick (m/s)
     * @param joy_angular_z Commanded angular.z from teleop stick (rad/s)
     * @param robot_pose Current pose (base_link; identity if unknown)
     * @param robot_speed Current body velocity for path scoring continuity
     * @param command_out Output TwistStamped in base_link
     * @return false if not configured or command_out is null
     */
    bool Tick(double joy_linear_x, double joy_angular_z,
              const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose,
              const automsgs::msgs::geometry_msgs::Twist& robot_speed,
              automsgs::msgs::geometry_msgs::TwistStamped* command_out);

    /**
     * @brief Whether assist is enabled in configuration
     */
    bool enabled() const { return options_.enabled; }

    /**
     * @brief Whether perception input is fresh enough for assisted driving
     */
    bool IsPerceptionOk() const;

    /**
     * @brief Whether any perception message has been received since configure
     */
    bool GotPerception() const;

    /**
     * @brief Identity pose in base_link (teleop plans in robot frame)
     * @param pose Output pose stamped
     * @return false if pose is null
     */
    bool TryGetRobotPose(automsgs::msgs::geometry_msgs::PoseStamped* pose) const;

    /**
     * @brief Publish empty path / free_paths so Autoviz can discover channels
     */
    void ClearPathViz();

    /**
     * @brief Republish path preview for late Autoviz subscribers
     */
    void RefreshPreview();

    /**
     * @brief Cache latest joystick values for the discovery publisher thread
     * @param joy_linear_x Latest linear stick value
     * @param joy_angular_z Latest angular stick value
     */
    void CacheJoy(double joy_linear_x, double joy_angular_z);

    /**
     * @brief Set max linear speed for CMU joySpeed normalization
     * @param max_linear_speed Must match TeleopClient::Configure value
     */
    void SetJoyMaxSpeed(double max_linear_speed);

private:
    /**
     * @brief Refresh rolling costmap and wrap it as a PathObstacleGrid
     */
    CostmapObstacleGrid RefreshGrid() const;

    /**
     * @brief Select paths and publish path + free_path_markers preview
     * @param joy_linear_x Latest linear stick value
     * @param joy_angular_z Latest angular stick value
     */
    void PublishPreview(double joy_linear_x, double joy_angular_z);

    /**
     * @brief Default rolling costmap options for teleop
     */
    static map::proto::Costmap2DOptions DefaultCostmapOptions(
        const LaserScanOptions& laser_scan,
        const PointCloudObstacleFeeder::Options& point_cloud,
        bool use_laser_for_costmap);

    /**
     * @brief True when LaserScan data is within stale_timeout_sec
     */
    bool IsScanFresh() const;

    /**
     * @brief Default MPPI controller options for teleop path following
     */
    static control::proto::MPPIControllerOptions DefaultMppiOptions();

    /**
     * @brief Map stick vector to intent heading in base_link (degrees)
     * @param angular_z Angular stick component (rad/s)
     * @param linear_x Linear stick component (m/s)
     */
    double JoyDirDeg(double angular_z, double linear_x) const;

    /**
     * @brief Write commanded twist to output without assist processing
     */
    void EmitPassthrough(double linear_x, double angular_z,
                            automsgs::msgs::geometry_msgs::TwistStamped* command_out) const;

    /**
     * @brief Ray-cast along joy_dir_deg for obstacles within probe range
     * @param grid Local obstacle grid in base_link
     * @param joy_dir_deg Stick heading (degrees)
     */
    bool IsForwardBlocked(const PathObstacleGrid& grid,
                                   double joy_dir_deg) const;

    /**
     * @brief True when forward stick is active but path/ray is blocked
     */
    bool NeedsBlockedTurn(const PathObstacleGrid& grid,
                             const PathSelectionResult& selection,
                             double joy_linear_x, double joy_angular_z) const;

    /**
     * @brief Command linear_x=0 and safe angular_z toward joystick heading
     */
    void EmitBlockedTurn(const PathObstacleGrid& grid,
                                double joy_linear_x, double joy_angular_z,
                                automsgs::msgs::geometry_msgs::TwistStamped*
                                    command_out) const;

    /**
     * @brief CMU: joySpeed = 0 when forward stick is centered
     * @param joy_linear_x Linear stick component (m/s)
     */
    bool HasForwardStick(double joy_linear_x) const;

    /**
     * @brief CMU joySpeed: hypot(linear, angular) / max_linear
     *
     * Returns 0 without forward stick.
     */
    double JoySpeedNorm(double joy_linear_x, double joy_angular_z) const;

    /**
     * @brief Run IntentPathSelector for current joystick intent
     */
    PathSelectionResult SelectPaths(const CostmapObstacleGrid& grid,
                                    double joy_linear_x, double joy_angular_z,
                                    const automsgs::msgs::geometry_msgs::Twist&
                                        robot_speed);

    /**
     * @brief Cache last path selection for the viz discovery thread
     */
    void UpdatePreviewCache(const PathSelectionResult& selection,
                            double joy_linear_x, double joy_angular_z);

    /**
     * @brief Invalidate cached preview (e.g. stick centered)
     */
    void InvalidatePreviewCache();

    // Cached path selection for visualization (updated in Tick).
    struct PreviewCache {
        PathSelectionResult selection;
        double joy_linear{0.0};
        double joy_angular{0.0};
        bool valid{false};
    };
    mutable std::mutex preview_mutex_;
    PreviewCache preview_cache_;

    // Loaded assist configuration.
    Options options_;
    // Autolink node for I/O.
    std::shared_ptr<autolink::Node> node_;
    // TF buffer shared with MPPI controller.
    std::shared_ptr<transform::Buffer> transform_buffer_;
    // Rolling local costmap wrapper.
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
    // Optional writer for local costmap visualization.
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        costmap_writer_;
    // RGB-D → costmap obstacle feeder.
    PointCloudObstacleFeeder feeder_;
    // Optional LaserScan reader for costmap.
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::LaserScan>>
        scan_reader_;
    // Timestamp of last LaserScan message.
    std::chrono::steady_clock::time_point last_scan_time_{};
    // CMU path library and selector.
    IntentPathSelector selector_;
    // Path / marker publisher for Autoviz.
    TeleopPathVisualizer path_visualizer_;
    // MPPI controller for follow-path tracking.
    std::unique_ptr<control::controller::mppi_controller::MPPIController> mppi_;
    // Permissive goal checker (teleop has no explicit goal pose).
    control::checker::SimpleGoalChecker goal_checker_;
    // True after successful Configure().
    bool configured_{false};
    // Robot footprint radius used for blocked-turn probing (m).
    double robot_radius_{0.22};
    // Latest stick values for discovery publisher thread.
    std::atomic<double> last_joy_linear_{0.0};
    std::atomic<double> last_joy_angular_{0.0};
    // Max linear speed for joySpeed normalization (from TeleopClient).
    std::atomic<double> joy_max_linear_speed_{0.0};
};

}  // namespace autonomy::task::teleop
