/* Copyright 2026 The Openbot Authors. */
/** @file exploration_test_support.hpp
 * @brief Real exploration consumers with explicitly controlled service replies.
 * Mapping receipts in this fixture do not establish actual mapper fusion.
 */
#pragma once
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include "autonomy/task/exploration/exploration_session.hpp"
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"
#include "autonomy/task/exploration/observation/camera_visibility_model.hpp"
namespace exploration_test {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace v = autonomy::planning::planner::vista;
namespace n = autonomy::navigation;
inline void Require(bool ok, const char* message) {
    if (!ok)
        throw std::runtime_error(message);
}

class OneGoal final : public e::ExplorationStrategy
{
public:
    explicit OneGoal(std::vector<double> views, e::Pose2d pose = {1, 1, 0})
        : views_(std::move(views)), pose_(pose) {}
    void Reset() override {
        selected_ = false;
    }
    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
                               const e::MapReadView& map,
                               const e::CoverageState& coverage, p::SensorMask,
                               double,
                               const std::function<bool()>& canceled) override {
        e::StrategyDecision result;
        if (selected_ || (canceled && canceled()))
            return result;
        selected_ = true;
        e::ObservationGoal goal;
        goal.goal_id = 42;
        goal.goal_version = 1;
        goal.map_epoch = map.map_epoch;
        goal.coverage_revision = coverage.revision();
        goal.original_pose = pose_;
        goal.required_views = views_;
        goal.allowed_translation = goal.allowed_yaw_change = 0;
        goal.required_coverage_units = {
            {10, 5, 2, 0, e::CoverageKind::kSurface}};
        goal.minimum_gain = .01;
        result.goal = std::move(goal);
        return result;
    }

private:
    std::vector<double> views_;
    e::Pose2d pose_;
    bool selected_ = false;
};

class EmptyComplete final : public e::ExplorationStrategy
{
public:
    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
                               const e::MapReadView&, const e::CoverageState&,
                               p::SensorMask, double,
                               const std::function<bool()>&) override {
        e::StrategyDecision result;
        result.grid_fallback_checked = result.sweep_checked = true;
        return result;
    }
};

class ThrowingStrategy final : public e::ExplorationStrategy
{
public:
    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
                               const e::MapReadView&, const e::CoverageState&,
                               p::SensorMask, double,
                               const std::function<bool()>&) override {
        throw 42;
    }
};

struct Fixture {
    n::GridMap grid{20, 20, .2};
    e::MapReader reader;
    std::shared_ptr<v::VistaRoutePlanner> planner;
    std::shared_ptr<e::VistaPathProvider> paths;
    std::vector<std::pair<e::Pose2d, e::RequestVersion>> submissions;
    std::optional<e::NavigationReply> reply;
    std::optional<e::RequestVersion> refresh_request, save_request;
    std::optional<e::MappingReply> refresh_reply, save_reply;
    std::optional<e::RequestVersion>
        startup_request;  ///< Actual startup phase request identity.
    std::optional<e::StartupObservationReply>
        startup_reply;  ///< Controlled native rotation result.
    std::size_t startup_cancellations =
        0;  ///< Matching versioned startup cancellation count.
    std::vector<e::RequestVersion>
        refresh_submissions;  ///< Every actual refresh callback identity.
    std::vector<e::RequestVersion>
        mapping_cancellations;  ///< Versioned mapping cancel callback
                                ///< identities.
    std::size_t cancellation_count = 0;
    std::size_t mapping_cancellation_count =
        0;  ///< Actual versioned mapping cancel callbacks.
    std::atomic<bool> hold_graph_updates{
        false};  ///< Keeps the real graph consumer behind mapper receipts.
    bool throw_poll = false, throw_cancel = false;
    std::uint64_t epoch = 0, sequence = 0;
    std::unique_ptr<e::ExplorationSession> session;

    /** @brief Creates real exploration consumers with controlled external
     * callbacks.
     * @param configure Optional startup policy and budget overrides for fault
     * regressions.
     * @param seed_initial_input When false, omit synthetic initial pose and
     * observation so a real decoded packet must establish sensing and
     * localization readiness.
     */
    explicit Fixture(
        std::vector<double> views, double dwell = .05,
        std::shared_ptr<e::ExplorationStrategy> strategy = {},
        bool require_save = false, bool wait_for_navigation = true,
        double pose_age = 10, std::size_t event_budget = 32,
        e::ExplorationDriver driver = e::ExplorationDriver::kAutomatic,
        bool require_return = false,
        std::function<void(e::ExplorationSessionOptions&)> configure = {},
        bool seed_initial_input = true) {
        grid.Fill({n::CellState::kFree, 0});
        auto map = reader.Read(grid, 1, 0, 1);
        if (require_save) {
            map.mapping_synchronized = true;
            map.observation_cut.Include(p::SensorKind::kRgbd, 1);
        }
        epoch = map.map_epoch;
        n::SafetyOptions safety;
        safety.robot_radius = .1;
        safety.horizontal_margin = .01;
        planner = std::make_shared<v::VistaRoutePlanner>(safety);
        paths = std::make_shared<e::VistaPathProvider>(planner);
        p::RigidTransform extrinsics;
        extrinsics.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
        extrinsics.translation.z = .5;
        auto visibility = std::make_shared<e::CameraVisibilityModel>(
            p::CameraModel{80, 60, 80, 80, 40, 30, .1, 8}, extrinsics);
        auto evaluator = std::make_shared<e::ViewpointEvaluator>(
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
                visibility});
        auto adapter =
            std::make_shared<e::GoalPoseAdapter>(paths, evaluator, safety);
        e::ExplorationServices services;
        services.submit_navigation = [this](const auto& pose,
                                            const auto& version) {
            submissions.emplace_back(pose, version);
            return true;
        };
        services.poll_navigation = [this] {
            if (throw_poll)
                throw 43;
            auto result = reply;
            reply.reset();
            return result;
        };
        services.cancel_navigation = [this](const auto&) {
            ++cancellation_count;
            if (throw_cancel)
                throw 44;
        };
        services.request_map_refresh = [this](const auto& version) {
            refresh_submissions.push_back(version);
            refresh_request = version;
            return true;
        };
        services.poll_map_refresh = [this] {
            auto result = refresh_reply;
            refresh_reply.reset();
            return result;
        };
        services.save_map = [this](const auto&, const auto& version) {
            save_request = version;
            return true;
        };
        services.poll_map_save = [this] {
            auto result = save_reply;
            save_reply.reset();
            return result;
        };
        services.cancel_mapping = [this](const auto& version) {
            mapping_cancellations.push_back(version);
            ++mapping_cancellation_count;
        };
        services.update_route_map = [this](const auto& view) {
            if (!hold_graph_updates.load())
                planner->Update(view);
        };
        services.navigation_still_valid = [](const auto&, const auto&,
                                             const auto&) { return true; };
        services.request_startup_observation = [this](const auto& version,
                                                      double, double) {
            startup_request = version;
            return true;
        };
        services.poll_startup_observation = [this] {
            auto result = startup_reply;
            startup_reply.reset();
            return result;
        };
        services.cancel_startup_observation = [this](const auto&) {
            ++startup_cancellations;
        };
        e::ExplorationSessionOptions options;
        options.validated_controller_modes = {
            p::Mask(p::SensorMode::kRgbdOnly)};
        options.require_return = options.require_save = false;
        options.require_save = require_save;
        options.require_return = require_return;
        options.driver = driver;
        // Saving scenarios cover this fully observed room. Without an explicit
        // task boundary, the unknown space outside the fixture is still work.
        if (require_save || driver == e::ExplorationDriver::kBehaviorTree)
            options.allowed_region = e::Bounds2d{0, 0, 4, 4};
        options.completion_rounds = require_save ? 1 : 3;
        options.maximum_events_per_tick = event_budget;
        options.map_max_age = options.health.maximum_age = 10;
        options.pose_max_age = pose_age;
        options.health.recovery_hold = 0;
        options.minimum_observation_time = dwell;
        if (configure)
            configure(options);
        session = std::make_unique<e::ExplorationSession>(
            strategy ? std::move(strategy)
                     : std::make_shared<OneGoal>(std::move(views)),
            adapter, paths, services, options);
        session->Start({1, 1, 0}, epoch, 1);
        Require(session->PushMap(map), "initial map queued");
        e::CoverageUnit unit;
        unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
        unit.center = {2, 1, .5};
        unit.normal = {-1, 0, 0};
        unit.required_quality = .5;
        if (wait_for_navigation)
            Require(session->PushCoverageUnit(unit),
                    "coverage requirement queued");
        if (seed_initial_input) {
            Pose(1, 0);
            Observe(1, false);
        }
        if (!wait_for_navigation ||
            driver == e::ExplorationDriver::kBehaviorTree) {
            session->Tick(1.01);
            return;
        }
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (submissions.empty() &&
               std::chrono::steady_clock::now() < deadline) {
            session->Tick(1.01);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        Require(submissions.size() == 1,
                "one owner submits initial observation navigation");
    }

    void Pose(double stamp, double yaw) {
        PushPose({1, 1, yaw}, stamp);
    }
    void PushPose(const e::Pose2d& pose, double stamp) {
        Require(session->PushPose(pose, stamp, epoch,
                                  p::Mask(p::SensorKind::kRgbd)),
                "current pose queued");
    }
    void Observe(double stamp, bool target = true) {
        p::SensorObservation observation;
        observation.sensor = p::SensorKind::kRgbd;
        observation.sequence = ++sequence;
        observation.map_epoch = epoch;
        observation.stamp = stamp;
        observation.valid_until = stamp + 10;
        observation.valid_fraction = 1;
        observation.localization_valid = true;
        observation.rays.push_back(
            {{1, 1, .5},
             target ? p::Vec3{1, 0, 0} : p::Vec3{0, 1, 0},
             target ? 1.0 : .2,
             true,
             1,
             stamp});
        Require(session->PushObservation(std::move(observation), stamp),
                "valid observation queued");
    }
    /** @brief Advance the producer ordinal without admitting the skipped frame.
     */
    void SkipObservation() {
        ++sequence;
    }
    void Arrive(double stamp) {
        for (std::size_t leg = 0; leg < 16; ++leg) {
            const auto submitted = submissions.size();
            Require(session->PushPose(submissions.back().first, stamp, epoch,
                                      p::Mask(p::SensorKind::kRgbd)),
                    "commanded arrival pose queued");
            reply = e::NavigationReply{submissions.back().second, true};
            auto state = session->Tick(stamp);
            if (state.state == e::ExplorationState::kObserving)
                return;
            for (std::size_t wait = 0;
                 wait < 200 && submissions.size() == submitted; ++wait) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                stamp += 1e-5;
                state = session->Tick(stamp);
                if (state.state == e::ExplorationState::kObserving)
                    return;
            }
            if (submissions.size() == submitted)
                throw std::runtime_error(
                    "successful route waypoint continues toward observation: " +
                    state.reason);

            stamp += 1e-3;
        }
        Require(false, "bounded route continuation reaches observation");
    }
};

}  // namespace exploration_test
