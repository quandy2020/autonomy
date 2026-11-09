/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/planning/plugins/policy/simulation/policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>

#include "autonomy/common/helper_functions/angle_utils.hpp"
#include "autonomy/common/math/math.hpp"
#include "navigation/common/configuration_loader.hpp"
#include "navigation/common/pose2d.hpp"
#include "navigation/common/pose2d_convert.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/tasks/frontier_detector_task.hpp"

namespace navigation {
namespace planning {
namespace simulation {
namespace {

/** @brief Manifest used when ``policy.frontiernet`` omits a path. */
constexpr char kDefaultFrontierManifest[] = "conf/models/frontiernet_defm.pb.txt";

NavigationIntent MakeIntent(const common::Twist2D& command) {
    NavigationIntent intent;
    auto* chunk = intent.mutable_chunk();
    chunk->set_dt_s(0.1);
    for (int i = 0; i < 4; ++i) {
        auto* action = chunk->add_actions();
        action->set_x(command.x());
        action->set_y(command.y());
        action->set_theta(command.theta());
    }
    intent.set_confidence(0.6F);
    return intent;
}

common::Twist2D Seek(const common::Pose2D& current,
                     const common::Pose2D& target, double max_linear_mps,
                     double position_tolerance) {
    const common::Pose2D relative =
        common::Compose(common::Inverse(current), target);
    const double distance = std::hypot(relative.x(), relative.y());
    common::Twist2D command;
    if (distance <= position_tolerance) {
        command.set_x(0.0F);
        const double heading_error =
            autonomy::common::helper_functions::wrap_angle(
                target.theta() - current.theta());
        command.set_theta(static_cast<float>(
            autonomy::common::Clamp(3.0 * heading_error, -0.8, 0.8)));
        return command;
    }
    const double heading_error = std::atan2(relative.y(), relative.x());
    double omega =
        autonomy::common::Clamp(1.5 * heading_error, -0.8, 0.8);
    if (std::abs(heading_error) > 0.4) {
        if (std::abs(omega) < 0.45) {
            omega = std::copysign(0.45, heading_error);
        }
        command.set_x(0.0F);
        command.set_theta(static_cast<float>(omega));
        return command;
    }
    command.set_x(static_cast<float>(
        autonomy::common::Clamp(0.6 * distance, 0.0, max_linear_mps) *
        std::max(0.0, std::cos(heading_error))));
    command.set_theta(static_cast<float>(omega));
    return command;
}

/** @brief Scan without translation; Safety still checks footprint rotation. */
common::Twist2D Spin() {
    common::Twist2D command;
    command.set_theta(0.55F);
    return command;
}

/**
 * @brief Nearest valid depth over the central image band.
 *
 * @details The straight-line seek controller has no obstacle avoidance, so a
 * blocked front means the robot would otherwise push into the obstacle until
 * the safety shield stops it. Returns 0 when the depth image is unusable.
 */
double FrontClearance(const observation::Observation& observation) {
    if (!observation.has_depth()) {
        return 0.0;
    }
    const auto& image = observation.depth();
    if (image.encoding() != "32FC1" || image.width() == 0 ||
        image.height() == 0) {
        return 0.0;
    }
    const std::size_t count =
        static_cast<std::size_t>(image.width()) * image.height();
    if (count > image.data().size() / sizeof(float)) {
        return 0.0;
    }

    const int u0 = static_cast<int>(image.width()) * 2 / 5;
    const int u1 = static_cast<int>(image.width()) * 3 / 5;
    const int v0 = static_cast<int>(image.height()) * 2 / 5;
    const int v1 = static_cast<int>(image.height()) * 3 / 5;
    double nearest = 0.0;
    for (int v = v0; v < v1; ++v) {
        for (int u = u0; u < u1; ++u) {
            float value = 0;
            std::memcpy(&value, image.data().data() +
                (static_cast<std::size_t>(v) * image.width() + u) * sizeof(float), sizeof(float));
            const double depth = value;
            if (!std::isfinite(depth) || depth <= 0.05) {
                continue;
            }
            if (nearest == 0.0 || depth < nearest) {
                nearest = depth;
            }
        }
    }
    return nearest;
}

/** @brief Median valid depth in the left or right third of the image. */
double SideClearance(const observation::Observation& observation, bool left)
{
    if (!observation.has_depth()) {
        return 0.0;
    }
    const auto& image = observation.depth();
    if (image.encoding() != "32FC1" || image.width() == 0 || image.height() == 0) {
        return 0.0;
    }
    const int u0 = left ? 0 : static_cast<int>(image.width()) * 2 / 3;
    const int u1 = left ? static_cast<int>(image.width()) / 3
                        : static_cast<int>(image.width());
    const int v0 = static_cast<int>(image.height()) * 2 / 5;
    const int v1 = static_cast<int>(image.height()) * 3 / 5;
    double nearest = 0.0;
    for (int v = v0; v < v1; ++v) {
        for (int u = u0; u < u1; ++u) {
            float value = 0;
            std::memcpy(&value, image.data().data() +
                (static_cast<std::size_t>(v) * image.width() + u) * sizeof(float),
                sizeof(float));
            if (!std::isfinite(value) || value <= 0.05) {
                continue;
            }
            if (nearest == 0.0 || value < nearest) {
                nearest = value;
            }
        }
    }
    return nearest;
}

/**
 * @brief Nearest depth over the low band the chassis sweeps.
 *
 * @details The central band misses table and chair legs, which sit below the
 * camera axis; the robot then drives until the safety shield reports an
 * infeasible footprint and stops for good. Rows 55-95% cover that blind spot
 * while staying above the floor return, which is always farther than 0.45 m.
 */
double LowClearance(const observation::Observation& observation)
{
    if (!observation.has_depth()) {
        return 0.0;
    }
    const auto& image = observation.depth();
    if (image.encoding() != "32FC1" || image.width() == 0 || image.height() == 0) {
        return 0.0;
    }
    const int u0 = static_cast<int>(image.width()) / 4;
    const int u1 = static_cast<int>(image.width()) * 3 / 4;
    const int v0 = static_cast<int>(image.height()) * 55 / 100;
    const int v1 = static_cast<int>(image.height()) * 95 / 100;
    double nearest = 0.0;
    for (int v = v0; v < v1; ++v) {
        for (int u = u0; u < u1; ++u) {
            float value = 0;
            std::memcpy(&value, image.data().data() +
                (static_cast<std::size_t>(v) * image.width() + u) * sizeof(float),
                sizeof(float));
            if (!std::isfinite(value) || value <= 0.05) {
                continue;
            }
            if (nearest == 0.0 || value < nearest) {
                nearest = value;
            }
        }
    }
    return nearest;
}

/**
 * @brief True when translating straight ahead would hit something.
 *
 * @details Combines the central band (walls, furniture) with the low band
 * (legs) so a blocked front is detected before the footprint is in contact.
 */
bool PathBlocked(const observation::Observation& observation)
{
    const double front = FrontClearance(observation);
    if (front <= 0.0) {
        // No usable depth: the caller decides. Exploration fails closed, a pose
        // goal keeps the previous "no sensor, no veto" behaviour.
        return false;
    }
    if (front < 0.8) {
        return true;
    }
    const double low = LowClearance(observation);
    return low > 0.0 && low < 0.45;
}

common::Twist2D ApplyFrontClearance(const observation::Observation& observation,
                                    common::Twist2D command)
{
    if (PathBlocked(observation) && command.x() > 0.0F) {
        command.set_x(0.0F);
        const double left = SideClearance(observation, true);
        const double right = SideClearance(observation, false);
        if (std::abs(command.theta()) < 0.2F) {
            const double open = left >= right ? 0.45 : -0.45;
            command.set_theta(static_cast<float>(open));
        }
    }
    return command;
}

common::Pose2D SteeringAlong(const std::vector<Waypoint>& path,
                             const common::Pose2D& current)
{
    common::Pose2D steering = path.front().pose;
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i >= 2) {
            const double ax = path[i - 1].pose.x() - path[i - 2].pose.x();
            const double ay = path[i - 1].pose.y() - path[i - 2].pose.y();
            const double bx = path[i].pose.x() - path[i - 1].pose.x();
            const double by = path[i].pose.y() - path[i - 1].pose.y();
            if (std::abs(ax * by - ay * bx) > 1e-6) {
                break;
            }
        }
        steering = path[i].pose;
        if (common::Distance(current, steering) >= 0.35) {
            break;
        }
    }
    return steering;
}

common::Twist2D NavigatePose(const observation::Observation& observation,
                             const common::Pose2D& current,
                             const common::Pose2D& target, double max_linear_mps,
                             double position_tolerance, double clearance_m)
{
    common::Pose2D steering = target;
    if ((observation.validity() & observation::OBSERVATION_FLAG_MAP_VALID) &&
        observation.has_map() &&
        observation.map().header().frame_id() == observation.map_frame()) {
        FrontierDetectorTask detector;
        const auto path = detector.PathTo(observation.map(), current, target, clearance_m);
        if (!path.empty()) {
            steering = SteeringAlong(path, current);
        }
    }
    return ApplyFrontClearance(observation,
                               Seek(current, steering, max_linear_mps, position_tolerance));
}

/** @brief Translate the proto options into the frontier source options. */
FrontierNetFrontierOptions MakeFrontierOptions(
    const config::FrontierNetOptions& options) {
    FrontierNetFrontierOptions result;
    if (options.df_normalizer() > 0.0) {
        result.df_normalizer = options.df_normalizer();
    }
    if (options.df_threshold() > 0.0) {
        result.df_threshold = options.df_threshold();
    }
    if (options.max_range() > 0.0) {
        result.max_range = options.max_range();
    }
    if (options.min_gain() > 0.0) {
        result.min_gain = options.min_gain();
    }
    if (options.min_cluster_pixels() > 0) {
        result.min_cluster_pixels = options.min_cluster_pixels();
    }
    if (options.target_distance() > 0.0) {
        result.target_distance = options.target_distance();
    }
    result.log_frontiers = options.log_frontiers();
    return result;
}

}  // namespace

Policy::Policy(plugins::VintPolicyOptions options,
               std::function<std::chrono::steady_clock::time_point()> now)
    : now_(now ? std::move(now) : [] { return std::chrono::steady_clock::now(); }) {
    (void)options;
}

bool Policy::Configure(const config::PolicyOptions& options) {
    if (options.name() != "simulation" && options.name() != "vint") {
        return false;
    }
    configured_ = false;
    Reset();
    exploration_options_ = options.exploration();
    clearance_m_ = exploration_options_.clearance_m() > 0 ? exploration_options_.clearance_m() : 0.4;
    for (const double value : {exploration_options_.clearance_m(), exploration_options_.max_range_m(),
             exploration_options_.no_progress_timeout_s(), exploration_options_.target_timeout_s(),
             exploration_options_.history_ttl_s()})
        if (!std::isfinite(value) || value < 0) return false;
    if (exploration_options_.max_history_entries() > 1024) return false;
    frontier_source_.reset();
    cache_valid_ = false;
    if (!options.has_frontiernet() || !options.frontiernet().enabled()) {
        configured_ = true;
        return true;
    }
    frontiernet_options_ = options.frontiernet();
    config::ModelManifest manifest;
    const std::string path = frontiernet_options_.manifest_path().empty()
                                 ? kDefaultFrontierManifest
                                 : frontiernet_options_.manifest_path();
    if (!common::LoadTextProto(path, &manifest).Ok()) {
        return false;
    }
    std::string error;
    auto source = std::make_unique<FrontierNetFrontierSource>(
        MakeFrontierOptions(frontiernet_options_));
    if (!source->LoadModel(manifest, &error)) {
        return false;
    }
    frontier_source_ = std::move(source);
    configured_ = true;
    return true;
}

std::vector<Waypoint> Policy::ExplorationFrontiers(
    const observation::Observation& observation, const common::Pose2D& current) {
    // The map is authoritative while it still has candidates: model projections
    // cannot override occupied or unreachable cells. Once the map offers none
    // (empty, stale, or already explored) the depth model takes over; its
    // waypoints are still validated against the map by the caller.
    if (observation.has_map()) {
        FrontierDetectorTask detector;
        auto frontiers = detector.Detect(observation.map(), current,
            exploration_options_.max_range_m() > 0 ? exploration_options_.max_range_m() : 6.0,
            clearance_m_);
        if (!frontiers.empty()) {
            frontier_source_kind_ = FrontierSourceKind::kMap;
            return frontiers;
        }
    }
    if (frontier_source_ != nullptr && frontier_source_->Ready()) {
        const auto now = now_();
        const auto period = std::chrono::milliseconds(
            frontiernet_options_.inference_period_ms() > 0
                ? frontiernet_options_.inference_period_ms()
                : 500);
        const bool moved =
            cache_valid_ && (common::Distance(current, cached_pose_) > 0.5 ||
                std::abs(autonomy::common::helper_functions::wrap_angle(current.theta() - cached_pose_.theta())) > 0.3);
        if (!cache_valid_ || moved || now - last_inference_ >= period) {
            cached_frontiers_ = frontier_source_->Detect(observation, current);
            cached_pose_ = current;
            last_inference_ = now;
            cache_valid_ = true;
        }
        frontier_source_kind_ = FrontierSourceKind::kModel;
        return cached_frontiers_;
    }
    frontier_source_kind_ = FrontierSourceKind::kNone;
    return {};
}

void Policy::SetFrontierSourceForTest(std::unique_ptr<FrontierSourceInterface> source) {
    frontier_source_ = std::move(source);
    cache_valid_ = false;
    cached_frontiers_.clear();
    frontier_source_kind_ = FrontierSourceKind::kNone;
}

common::StatusOr<NavigationIntent> Policy::Query(
    const observation::Observation& observation,
    const observation::DepthFeature& depth_feature, const GoalSpec& goal,
    const memory::ContextView& context) {
    (void)depth_feature;
    (void)context;
    if (!configured_) {
        return common::Status::Unavailable(
            "simulation policy is not configured");
    }
    common::Pose2D current;
    if (observation.has_odom() && observation.odom().has_pose()) {
        current = common::FromProto(observation.odom().pose().pose());
    }
    common::Pose2D target;
    if (goal.has_pose()) {
        target = common::FromProto(goal.pose().pose());
    } else if (goal.type() == GoalSpec::TYPE_EXPLORATION) {
        if (!observation.has_odom() || !observation.odom().has_pose() ||
            !common::IsFinite(current)) return MakeIntent(common::Twist2D{});
        return MakeIntent(Explore(observation, current));
    } else {
        return MakeIntent(Spin());
    }
    const double tolerance =
        goal.position_tolerance() > 0.0 ? goal.position_tolerance() : 0.15;
    return MakeIntent(
        NavigatePose(observation, current, target, 0.4, tolerance, clearance_m_));
}

void Policy::LogMode(const char* mode) {
    if (mode_ == mode) return;
    mode_ = mode;
    if (exploration_options_.log_transitions()) {
        std::fprintf(stderr, "[exploration] mode=%s history=%zu", mode, history_.size());
        if (target_) std::fprintf(stderr, " target=(%.3f,%.3f)", target_->x(), target_->y());
        std::fprintf(stderr, "\n");
    }
}

void Policy::Remember(const common::Pose2D& pose, bool failed) {
    const double ttl = exploration_options_.history_ttl_s() > 0 ? exploration_options_.history_ttl_s() : 120;
    history_.push_back({pose, now_() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(ttl)), failed});
    const auto limit = exploration_options_.max_history_entries() > 0 ? exploration_options_.max_history_entries() : 64;
    while (history_.size() > limit) history_.pop_front();
}

common::Twist2D Policy::Explore(const observation::Observation& observation,
                                const common::Pose2D& current) {
    const auto now = now_();
    while (!history_.empty() && history_.front().expires <= now) history_.pop_front();
    const bool mapped = observation.has_map();
    if (mapped) {
        if (!(observation.validity() & observation::OBSERVATION_FLAG_MAP_VALID)) {
            target_.reset(); LogMode("invalid_map"); return {};
        }
        // Include metadata as well as occupancy; exclude publication stamps.
        std::uint64_t hash = 1469598103934665603ULL;
        const auto mix = [&](const std::string& bytes) {
            for (unsigned char c : bytes) { hash ^= c; hash *= 1099511628211ULL; }
        };
        auto info = observation.map().info(); info.clear_map_load_time();
        mix(info.SerializeAsString()); mix(observation.map().header().frame_id());
        for (auto value : observation.map().data()) { hash ^= std::uint32_t(value); hash *= 1099511628211ULL; }
        if (hash != map_hash_) { map_hash_ = hash; scan_done_ = false; }
        // The current pose contract is map-frame; do not silently consume another grid frame.
        if (observation.map().header().frame_id() != observation.map_frame()) {
            target_.reset(); LogMode("map_frame_mismatch"); return {};
        }
    }
    FrontierDetectorTask detector;
    std::vector<Waypoint> route;
    const auto remaining_distance = [&](const std::vector<Waypoint>& path) {
        double length = 0;
        auto previous = current;
        for (const auto& point : path) { length += common::Distance(previous, point.pose); previous = point.pose; }
        return length;
    };
    if (target_) {
        if (mapped) route = detector.PathTo(observation.map(), current, *target_, clearance_m_);
        const double distance = common::Distance(current, *target_);
        const double progress = mapped && !route.empty() ? remaining_distance(route) : distance;
        // Path length falls while taking a detour even if Euclidean goal distance grows.
        if (progress < best_distance_ - 0.05) { best_distance_ = progress; progress_time_ = now; }
        const double idle = exploration_options_.no_progress_timeout_s() > 0 ? exploration_options_.no_progress_timeout_s() : 8;
        const double budget = exploration_options_.target_timeout_s() > 0 ? exploration_options_.target_timeout_s() : 45;
        const bool reached = distance <= 0.3;
        const bool failed = std::chrono::duration<double>(now - progress_time_).count() >= idle ||
                            std::chrono::duration<double>(now - target_started_).count() >= budget;
        if (reached || failed) {
            Remember(*target_, failed && !reached); LogMode(reached ? "reached" : "abandoned");
            target_.reset(); scanning_ = false; scan_done_ = false;
        }
    }
    if (!target_) {
        const auto candidates = ExplorationFrontiers(observation, current);
        for (const auto& candidate : candidates) {
            if (common::Distance(candidate.pose, current) < 0.3) continue;
            const bool seen = std::any_of(history_.begin(), history_.end(), [&](const auto& visit) {
                return common::Distance(candidate.pose, visit.pose) < (visit.failed ? 0.8 : 0.5);
            });
            if (seen) continue;
            if (mapped) {
                route = detector.PathTo(observation.map(), current, candidate.pose, clearance_m_);
                // A depth waypoint lies on a free ray, so a missing map path
                // (usually the start cell falling inside the clearance
                // inflation) must not discard it; map goals still need a path.
                if (route.empty() && frontier_source_kind_ != FrontierSourceKind::kModel) continue;
            }
            target_ = candidate.pose; target_started_ = now; progress_time_ = now;
            target_source_ = frontier_source_kind_;
            best_distance_ = mapped && !route.empty() ? remaining_distance(route)
                                                      : common::Distance(current, *target_);
            scanning_ = false; scan_done_ = false;
            LogMode(frontier_source_kind_ == FrontierSourceKind::kMap ? "map_target"
                                                                     : "model_target");
            break;
        }
    }
    if (target_) {
        common::Pose2D steering = *target_;
        if (mapped) {
            const auto& path = route;
            if (path.empty() && target_source_ != FrontierSourceKind::kModel) {
                Remember(*target_, true); LogMode("unreachable"); target_.reset();
                return Spin();
            }
            // A depth waypoint sits on a free ray, so it stays usable when the
            // map planner rejects the robot's own (inflated) start cell.
            if (path.empty()) LogMode("model_direct");
            else steering = SteeringAlong(path, current);
        }
        // A blocked front (wall or low obstacle) turns toward the more open side
        // instead of grinding into it; unusable depth never translates.
        auto command = Seek(current, steering, 0.3, 0.05);
        if (FrontClearance(observation) <= 0) { command.set_x(0); return command; }
        return ApplyFrontClearance(observation, command);
    }
    if (!scan_done_) {
        if (!scanning_) { scanning_ = true; scan_yaw_ = current.theta(); scan_rotation_ = 0; }
        const double delta = autonomy::common::helper_functions::wrap_angle(current.theta() - scan_yaw_);
        // Signed accumulation does not mistake an oscillating chassis for a full scan.
        scan_rotation_ += delta; scan_yaw_ = current.theta();
        if (scan_rotation_ < 6.0) { LogMode("scan"); return Spin(); }
        scanning_ = false; scan_done_ = true;
    }
    // A valid map with no map frontier and no model frontier means explored.
    if (mapped) { LogMode("no_frontier"); return {}; }
    // No rear sensor: translation is forward only, with conservative front depth.
    if (FrontClearance(observation) < 1.0) { LogMode("blocked_scan"); return Spin(); }
    common::Twist2D command; command.set_x(0.2F); command.set_theta(0.2F);
    LogMode("depth_cruise"); return command;
}

void Policy::Reset() {
    cache_valid_ = false;
    cached_frontiers_.clear();
    history_.clear(); target_.reset();
    scanning_ = false; scan_done_ = false;
    map_hash_ = 0; mode_.clear();
    frontier_source_kind_ = FrontierSourceKind::kNone;
    target_source_ = FrontierSourceKind::kNone;
}

std::string_view Policy::Name() const {
    return "simulation";
}

}  // namespace simulation
}  // namespace planning
}  // namespace navigation
