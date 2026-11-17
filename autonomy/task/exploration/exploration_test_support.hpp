/* Copyright 2026 The Openbot Authors. */
#pragma once
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
namespace n = opt::navigation;
inline void Require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

class OneGoal final : public e::ExplorationStrategy {
public:
    explicit OneGoal(std::vector<double> views) : views_(std::move(views)) {}
    void Reset() override { selected_ = false; }
    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
        const e::MapReadView& map, const e::CoverageState& coverage,
        p::SensorMask, double, const std::function<bool()>& canceled) override {
        e::StrategyDecision result;
        if (selected_ || (canceled && canceled())) return result;
        selected_ = true;
        e::ObservationGoal goal;
        goal.goal_id = 42;
        goal.goal_version = 1;
        goal.map_epoch = map.map_epoch;
        goal.coverage_revision = coverage.revision();
        goal.original_pose = {1, 1, 0};
        goal.required_views = views_;
        goal.allowed_translation = goal.allowed_yaw_change = 0;
        goal.required_coverage_units = {{10, 5, 2, 0, e::CoverageKind::kSurface}};
        goal.minimum_gain = .01;
        result.goal = std::move(goal);
        return result;
    }
private:
    std::vector<double> views_;
    bool selected_ = false;
};

class EmptyComplete final : public e::ExplorationStrategy {
public:
    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
        const e::MapReadView&, const e::CoverageState&, p::SensorMask, double,
        const std::function<bool()>&) override {
        e::StrategyDecision result;
        result.grid_fallback_checked = result.sweep_checked = true;
        return result;
    }
};

class ThrowingStrategy final : public e::ExplorationStrategy {
public:
    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
        const e::MapReadView&, const e::CoverageState&, p::SensorMask, double,
        const std::function<bool()>&) override { throw 42; }
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
    std::size_t cancellation_count = 0;
    bool throw_poll = false, throw_cancel = false;
    std::uint64_t epoch = 0, sequence = 0;
    std::unique_ptr<e::ExplorationSession> session;

    explicit Fixture(std::vector<double> views, double dwell = .05,
        std::shared_ptr<e::ExplorationStrategy> strategy = {}, bool require_save = false,
        bool wait_for_navigation = true, double pose_age = 10, std::size_t event_budget = 32,
        e::ExplorationDriver driver = e::ExplorationDriver::kAutomatic, bool require_return = false) {
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
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{visibility});
        auto adapter = std::make_shared<e::GoalPoseAdapter>(paths, evaluator, safety);
        e::ExplorationServices services;
        services.submit_navigation = [this](const auto& pose, const auto& version) {
            submissions.emplace_back(pose, version);
            return true;
        };
        services.poll_navigation = [this] {
            if (throw_poll) throw 43;
            auto result = reply;
            reply.reset();
            return result;
        };
        services.cancel_navigation = [this](const auto&) {
            ++cancellation_count;
            if (throw_cancel) throw 44;
        };
        services.request_map_refresh = [this](const auto& version) {
            refresh_request = version; return true;
        };
        services.poll_map_refresh = [this] {
            auto result = refresh_reply; refresh_reply.reset(); return result;
        };
        services.save_map = [this](const auto&, const auto& version) {
            save_request = version; return true;
        };
        services.poll_map_save = [this] {
            auto result = save_reply; save_reply.reset(); return result;
        };
        services.cancel_mapping = [](const auto&) {};
        services.update_route_map = [this](const auto& view) { planner->Update(view); };
        services.navigation_still_valid = [](const auto&, const auto&, const auto&) { return true; };
        e::ExplorationSessionOptions options;
        options.validated_controller_modes = {p::Mask(p::SensorMode::kRgbdOnly)};
        options.require_return = options.require_save = false;
        options.require_save = require_save;
        options.require_return = require_return;
        options.driver = driver;
        // Saving scenarios cover this fully observed room. Without an explicit
        // task boundary, the unknown space outside the fixture is still work.
        if (require_save || driver == e::ExplorationDriver::kBehaviorTree) options.allowed_region = e::Bounds2d{0, 0, 4, 4};
        options.completion_rounds = require_save ? 1 : 3;
        options.maximum_events_per_tick = event_budget;
        options.map_max_age = options.health.maximum_age = 10;
        options.pose_max_age = pose_age;
        options.health.recovery_hold = 0;
        options.minimum_observation_time = dwell;
        session = std::make_unique<e::ExplorationSession>(
            strategy ? std::move(strategy) : std::make_shared<OneGoal>(std::move(views)),
            adapter, paths, services, options);
        session->Start({1, 1, 0}, epoch, 1);
        Require(session->PushMap(map), "initial map queued");
        e::CoverageUnit unit;
        unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
        unit.center = {2, 1, .5};
        unit.normal = {-1, 0, 0};
        unit.required_quality = .5;
        if (wait_for_navigation) Require(session->PushCoverageUnit(unit), "coverage requirement queued");
        Pose(1, 0);
        Observe(1, false);
        if (!wait_for_navigation || driver == e::ExplorationDriver::kBehaviorTree) {
            session->Tick(1.01); return;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (submissions.empty() && std::chrono::steady_clock::now() < deadline) {
            session->Tick(1.01);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        Require(submissions.size() == 1, "one owner submits initial observation navigation");
    }

    void Pose(double stamp, double yaw) {
        Require(session->PushPose({1, 1, yaw}, stamp, epoch,
            p::Mask(p::SensorKind::kRgbd)), "current pose queued");
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
        observation.rays.push_back({{1, 1, .5}, target ? p::Vec3{1, 0, 0} : p::Vec3{0, 1, 0},
                                   target ? 1.0 : .2, true, 1, stamp});
        Require(session->PushObservation(std::move(observation), stamp), "valid observation queued");
    }
    void Arrive(double stamp) {
        Pose(stamp, submissions.back().first.yaw);
        reply = e::NavigationReply{submissions.back().second, true};
        Require(session->Tick(stamp).state == e::ExplorationState::kObserving,
                "arrival starts observation rather than completing coverage");
    }
};

}  // namespace exploration_test
