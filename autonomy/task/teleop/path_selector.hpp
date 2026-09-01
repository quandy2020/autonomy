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

#include <optional>
#include <vector>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/task/teleop/obstacle_grid.hpp"

namespace autonomy::task::teleop {

/**
 * @brief Configuration for CMU/FALCO polar spline path library generation
 */
struct PathLibraryOptions {
    // Use polar spline tree (path_generator.m); else arc library.
    bool use_polar_spline{true};
    // Base segment length for library paths (m).
    double segment_length{1.0};
    // CMU path_generator.m angle (deg); groups span ±this value.
    double max_heading_deg{27.0};
    // Hierarchy scale between spline tree levels.
    double hierarchy_scale{0.65};
    // CMU path length = 3 * segment_length unless overridden (m).
    double path_range{3.0};
    // Output path pose spacing (m).
    double sample_ds{0.05};
    // Knot spacing when building spline control points (m).
    double library_knot_ds{0.01};
    // RGB-D / sensor horizon; caps path_range at runtime (m).
    double sensor_range{0.0};
    // Follow group start path instead of best candidate geometry.
    bool use_group_start_path{true};
    // Multi-scale fallback minimum planning range (m).
    double min_path_range{1.0};
    // Step size when reducing path range on fallback (m).
    double path_range_step{0.5};
    // Carrot distance along selected path (m).
    double look_ahead_distance{0.5};
    // Aggregate path scores per polar group before picking best path.
    bool use_group_selection{true};
    // Lethal hits threshold to reject a path (CMU pointPerPathThre).
    int point_per_path_thr{2};
    // CMU local_planner dirWeight.
    double dir_weight{0.02};
    // CMU dirThre: intent half-width for forward paths (degrees).
    double dir_threshold_deg{90.0};
    // Scale planning horizon by |joy_speed| / max_linear_speed.
    bool path_range_by_speed{true};
    // Reference max linear speed for joySpeed scaling (m/s).
    double max_linear_speed{0.5};
    // CMU 36-direction rotation search (rotAng = 10*rotDir - 180 deg).
    bool use_rot_dir_search{true};
    // Number of rotDir bins (default 36).
    int num_rot_dirs{36};
    // Default path scale factor at full joy speed.
    double def_path_scale{1.25};
    // Minimum path scale during fallback.
    double min_path_scale{0.75};
    // Step when reducing path scale on fallback.
    double path_scale_step{0.25};
    // Scale path_scale by normalized joy speed.
    bool path_scale_by_speed{true};
    // CMU dirToVehicle: gate rotDir by vehicle-frame heading.
    bool dir_to_vehicle{false};
    // CMU checkRotObstacle: block rotDirs blocked by nearby obstacles.
    bool check_rot_obstacle{false};
    // Vehicle length for rot obstacle check (m).
    double rot_obstacle_vehicle_length{0.6};
    // Vehicle width for rot obstacle check (m).
    double rot_obstacle_vehicle_width{0.4};
    // Allow two-way drive rotDir alt angles.
    bool two_way_drive{true};
    // Multi-objective score: clearance penalty weight.
    double clearance_weight{0.35};
    // Multi-objective score: smoothness penalty weight.
    double smoothness_weight{0.15};
    // Multi-objective score: efficiency penalty weight.
    double efficiency_weight{0.1};
    // Multi-objective score: velocity continuity weight.
    double velocity_continuity_weight{0.2};
    // Multi-objective score: temporal hysteresis weight.
    double temporal_weight{0.25};
    // Divide group_rot_scores by paths scored per group.
    bool normalize_group_scores{true};
    // FALCO-style costmap traversability boost weight.
    double traversability_weight{0.25};
    // CMU PLOTPATHSET: free_paths only at selected rotDir.
    bool plot_path_set{true};
    // RGB-D horizontal FoV (deg); gate free paths to ±hfov/2.
    double rgbd_hfov_deg{90.0};
    // Camera optical center in base_link (m).
    double rgbd_camera_offset_x{0.03};
    double rgbd_camera_offset_y{0.0};
    // free_path_markers in base_link (not rotated by best_rot_deg).
    bool free_paths_in_base_link{true};
    // Show full symmetric path-library fan; clip at obstacles.
    bool free_paths_plot_library_fan{true};
    // Max LINE_STRIP markers for library fan (343 = full CMU set).
    int free_paths_max_markers{343};
    // Hide paths with lethal hits >= point_per_path_thr.
    bool free_paths_filter_collisions{true};
    // Penalty weight for deviating from prior selection.
    double goal_cost{10.0};
    // Legacy arc library (use_polar_spline=false).
    int num_dirs{9};
    int num_lengths{3};
    double max_range{3.0};
    double ds{0.1};
};

// One precomputed library trajectory in base_link.
struct PathCandidate {
    // Global candidate index in library.
    int id{0};
    // Polar group id [0, 6].
    int group_id{0};
    // Endpoint heading relative to start (degrees).
    double end_dir_deg{0.0};
    // Discretized path poses in base_link.
    automsgs::msgs::nav_msgs::Path path;
};

// CMU checkRotObstacle: allowed rotation interval (degrees, base_link).
struct RotObstacleAngles {
    // Minimum clockwise rotation limit (deg).
    double min_cw{-180.0};
    // Minimum counter-clockwise rotation limit (deg).
    double min_ccw{180.0};
};

// Optional robot motion for velocity-continuity scoring.
struct PathSelectContext {
    // Current body linear velocity (m/s).
    double robot_vx{0.0};
    // Current body angular velocity (rad/s).
    double robot_wz{0.0};
};

// Previous selection for temporal hysteresis.
struct PathTemporalState {
    // Whether temporal state is initialized.
    bool valid{false};
    // Last selected rotDir index.
    int rot_dir{18};
    // Last selected polar group.
    int group_id{-1};
    // Last selected library candidate index.
    int index{-1};
};

// Output of IntentPathSelector::Select().
struct PathSelectionResult {
    // Best follow path in base_link (trimmed, rotated, scaled).
    std::optional<automsgs::msgs::nav_msgs::Path> best_path;
    // Index into candidates() for best_path geometry.
    int best_index{-1};
    // Polar group of best candidate.
    int best_group_id{-1};
    // CMU rotDir index [0, 35]; rotAng = 10*rotDir - 180 deg.
    int best_rot_dir{18};
    // Best rotDir angle applied to library (degrees).
    double best_rot_deg{0.0};
    // Path scale applied to selected trajectory.
    double best_path_scale{1.0};
    // Active planning range before scale (m).
    double active_range{0.0};
    // Trimmed arc length before scale (m).
    double scaled_range{0.0};
    // Composite selection score (lower is better).
    double best_score{0.0};
    // CMU PLOTPATHSET: collision-free indices at best_rot_dir.
    std::vector<int> free_path_indices;
    // All feasible candidate indices at current search settings.
    std::vector<int> feasible_indices;
    // Per-candidate composite scores (parallel to feasible_indices).
    std::vector<double> scores;
};

/**
 * @class teleop::IntentPathSelector
 * @brief CMU local_planner-style path library and intent-based selection
 *
 * Generates 343 polar spline candidates (7 groups × 49 paths), scores them
 * against joystick heading and local costmap, and returns the best follow path
 * in base_link. Supports 36-direction rotDir search and multi-scale fallback.
 */
class IntentPathSelector
{
public:
    /**
     * @brief Build path library from options (polar spline or legacy arcs)
     * @param options Library generation and selection parameters
     */
    void GenerateLibrary(const PathLibraryOptions& options);

    /**
     * @brief Build legacy arc fan library
     * @param num_dirs Number of heading bins
     * @param num_lengths Number of length samples per direction
     * @param max_range Maximum arc length (m)
     * @param ds Pose spacing along arcs (m)
     */
    void GenerateArcLib(int num_dirs, int num_lengths, double max_range,
                                double ds);

    /**
     * @brief Select best path for joystick intent
     * @param grid Obstacle view in base_link
     * @param joy_dir_deg Stick heading relative to robot forward (degrees)
     * @param joy_speed Normalized stick magnitude in [0, 1]
     * @param context Optional robot velocity for continuity scoring
     */
    PathSelectionResult Select(const PathObstacleGrid& grid, double joy_dir_deg,
                               double joy_speed,
                               const PathSelectContext& context = {}) const;

    /**
     * @brief Convenience wrapper over Costmap2D
     */
    std::optional<automsgs::msgs::nav_msgs::Path> Select(
        const map::costmap_2d::Costmap2D& costmap, double joy_dir_deg,
        double joy_speed, const PathSelectContext& context = {}) const;

    /**
     * @brief Convenience wrapper over OccupancyGrid
     */
    std::optional<automsgs::msgs::nav_msgs::Path> Select(
        const automsgs::msgs::map_msgs::OccupancyGrid& grid,
        double joy_dir_deg, double joy_speed,
        const PathSelectContext& context = {}) const;

    /**
     * @brief All library path candidates
     */
    const std::vector<PathCandidate>& candidates() const { return candidates_; }

    /**
     * @brief CMU group spine paths (7 start paths for fan visualization)
     */
    const std::vector<automsgs::msgs::nav_msgs::Path>& group_start_paths() const {
        return group_start_paths_;
    }

    /**
     * @brief Options used to generate the current library
     */
    const PathLibraryOptions& library_options() const { return library_options_; }

    /**
     * @brief Override lethal-hit threshold for path rejection
     */
    void set_point_per_path_thre(int value) { point_per_path_thre_ = value; }

    /**
     * @brief Override CMU dirWeight for intent scoring
     */
    void set_dir_weight(double value) { dir_weight_ = value; }

    /**
     * @brief Override temporal goal-cost weight
     */
    void set_goal_cost(double value) { goal_cost_ = value; }

    /**
     * @brief CMU local_planner: rotate path by rot_deg then scale (base_link)
     */
    static automsgs::msgs::nav_msgs::Path RotateAndScalePath(
        const automsgs::msgs::nav_msgs::Path& path, double rot_deg,
        double scale);

    /**
     * @brief Truncate path to max arc length from the start pose
     */
    static automsgs::msgs::nav_msgs::Path TrimPathToArcLength(
        const automsgs::msgs::nav_msgs::Path& path, double max_length);

    /**
     * @brief CMU /path: library candidate trimmed then rot+scale
     */
    static automsgs::msgs::nav_msgs::Path BuildFollowPath(
        const automsgs::msgs::nav_msgs::Path& library_path, double scaled_range,
        double rot_deg, double path_scale);

    /**
     * @brief Polar library group heading (deg) at first spline knot
     */
    static double GroupHeadingDeg(int group_id, double max_heading_deg);

    /**
     * @brief Gate candidate paths to RGB-D horizontal field of view
     */
    static bool InRgbdFov(
        const PathCandidate& candidate,
        const automsgs::msgs::nav_msgs::Path& world_path,
        const PathLibraryOptions& options);

    /**
     * @brief Count lethal costmap hits along a path (segment-sampled)
     */
    static int CountLethalHits(const PathObstacleGrid& grid,
                               const automsgs::msgs::nav_msgs::Path& path,
                               double sample_step = 0.05);

    /**
     * @brief CMU checkRotObstacle: angular limits of nearby obstacles (deg)
     */
    static RotObstacleAngles RotObstacleScan(
        const PathObstacleGrid& grid, double path_scale, double vehicle_length,
        double vehicle_width);

    /**
     * @brief CMU rotAng = 10 * rotDir - 180 (degrees)
     */
    static double RotDirToDeg(int rot_dir);

    /**
     * @brief True if rotDir is within joystick intent cone
     */
    static bool JoyRotGate(double joy_dir_deg, int rot_dir,
                                    double dir_threshold, bool dir_to_vehicle);

    /**
     * @brief True if rotDir rotation does not sweep through nearby obstacles
     */
    static bool RotDirClear(int rot_dir,
                                         const RotObstacleAngles& angles,
                                         bool check_rot_obstacle,
                                         bool two_way_drive);

    /**
     * @brief Closest rotDir matching joy heading and passing obstacle gate
     * @return rotDir index, or -1 if none is safe
     */
    static int BestRotDir(double joy_dir_deg,
                                  const RotObstacleAngles& angles,
                                  const PathLibraryOptions& options);

    /**
     * @brief Fill scale/range fields when Select() found no feasible path (viz only)
     */
    static void FillPreviewScales(PathSelectionResult* result,
                                            double joy_speed_norm,
                                            const PathLibraryOptions& options);

private:
    /**
     * @brief CMU rotDir weight for intent scoring
     */
    static double RotDirWeight(int rot_dir);

    /**
     * @brief CMU local_planner direction + rotDir intent score
     */
    static double PathIntentScore(double end_dir_deg, double joy_dir_deg,
                                        int rot_dir, double dir_weight);

    /**
     * @brief Multi-objective path score combining intent and costmap terms
     */
    static double CompositePathScore(
        double intent_score, const PathObstacleGrid& grid,
        const automsgs::msgs::nav_msgs::Path& world_path,
        const PathLibraryOptions& options, double active_range,
        const PathSelectContext& context, int rot_dir, int group_id,
        const PathTemporalState* last_selection);

    /**
     * @brief Fill free_path_indices for PLOTPATHSET visualization
     */
    static void FillFreeIndices(
        PathSelectionResult* result, const PathObstacleGrid& grid,
        const std::vector<PathCandidate>& candidates, double path_scale,
        double scaled_range, int lethal_threshold,
        const PathLibraryOptions& options);

    /**
     * @brief Wrap angle to (-180, 180] degrees
     */
    static double WrapDeg(double deg);

    /**
     * @brief Endpoint heading from start to (end_x, end_y) in degrees
     */
    static double EndDirectionDeg(double end_x, double end_y);

    /**
     * @brief Generate CMU polar spline path library
     */
    void BuildSplineLib(const PathLibraryOptions& options);

    /**
     * @brief Generate legacy arc fan library
     */
    void GenerateArcLibrary(int num_dirs, int num_lengths, double max_range,
                            double ds);

    // All library path candidates.
    std::vector<PathCandidate> candidates_;
    // Seven group spine paths for fan visualization.
    std::vector<automsgs::msgs::nav_msgs::Path> group_start_paths_;
    // Options used to build candidates_.
    PathLibraryOptions library_options_;
    // Last selection for temporal hysteresis.
    mutable PathTemporalState temporal_state_;
    // Runtime override for lethal-hit threshold.
    int point_per_path_thre_{2};
    // Runtime override for dirWeight.
    double dir_weight_{0.02};
    // Runtime override for temporal goal cost.
    double goal_cost_{10.0};
};

}  // namespace autonomy::task::teleop
