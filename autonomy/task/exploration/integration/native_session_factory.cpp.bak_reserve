/* Copyright 2026 The Openbot Authors. */
/** @file native_session_factory.cpp
 * @brief Owner-executed assembly and acquisition/coverage calibration consistency.
 */
#include "autonomy/task/exploration/integration/native_session_factory.hpp"

#include <cmath>
#include <stdexcept>
#include "autonomy/task/exploration/observation/camera_visibility_model.hpp"
#include "autonomy/task/exploration/observation/lidar_visibility_model.hpp"
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"

namespace autonomy::task::exploration {
namespace {

/** @brief Build only visibility models supported by the configured physical sensors. */
std::shared_ptr<ViewpointEvaluator> MakeEvaluator(const NativeExplorationConfiguration& config) {
    std::vector<std::shared_ptr<const SensorVisibilityModel>> models;
    if (config.camera) models.push_back(std::make_shared<CameraVisibilityModel>(
        config.camera->camera, config.camera->base_from_camera));
    if (config.lidar) models.push_back(std::make_shared<LidarVisibilityModel>(
        config.lidar->beam_directions, config.lidar->angular_tolerance,
        config.input.decoding.minimum_range, config.input.decoding.maximum_range,
        config.lidar->base_from_lidar));
    return std::make_shared<ViewpointEvaluator>(std::move(models));
}

/** @brief Reject a split mode/calibration before native callbacks or factories can run. */
void Validate(const NativeExplorationConfiguration& config, const NativeExplorationPlatform& platform) {
    perception::ValidateNativeObservationSourceOptions(config.input);
    const auto mode = config.session.mode;
    const bool depth = (perception::Mask(mode) & perception::Mask(perception::SensorKind::kRgbd)) != 0;
    const bool lidar = (perception::Mask(mode) & perception::Mask(perception::SensorKind::kLidar)) != 0;
    if (config.input.mode != mode || config.coverage.mode != mode ||
        config.camera.has_value() != depth || config.lidar.has_value() != lidar ||
        config.session.validated_controller_modes.empty() ||
        !platform.create_mapping_services || !platform.localization || !platform.clock ||
        config.retirement_timeout <= std::chrono::milliseconds::zero() ||
        config.retirement_timeout > std::chrono::seconds(30))
        throw std::invalid_argument("native exploration requires consistent modes and actual platform services");
    const auto required = perception::Mask(mode);
    bool mode_validated = false;
    for (const auto supported : config.session.validated_controller_modes) {
        if (supported == 0 || supported > perception::Mask(perception::SensorMode::kLidarRgbd))
            throw std::invalid_argument("unsupported execution sensor mask");
        if (supported == required) mode_validated = true;
    }
    if (!mode_validated) throw std::invalid_argument("selected sensing mode has no validated native execution mode");
    if (config.routes.service_name.empty() || config.routes.planner_id.empty() ||
        config.routes.planner_id.size() > 256 || config.routes.frame_id.empty() ||
        config.routes.timeout.count() <= 0 || config.routes.timeout > std::chrono::seconds(5) ||
        config.routes.retry_interval.count() < 0 || config.routes.retry_interval > std::chrono::seconds(5) ||
        config.routes.maximum_poses < 2 || config.routes.maximum_poses > 2000000)
        throw std::invalid_argument("invalid native exploration route endpoint or budget");
    if (config.camera && (config.camera->camera.min_depth != config.input.minimum_depth ||
                          config.camera->camera.max_depth != config.input.maximum_depth))
        throw std::invalid_argument("coverage camera and native depth validity ranges differ");
    if (config.camera && config.input.expected_depth_camera) {
        const auto& expected = *config.input.expected_depth_camera; const auto& actual = config.camera->camera;
        if (expected.width != actual.width || expected.height != actual.height || expected.fx != actual.fx ||
            expected.fy != actual.fy || expected.cx != actual.cx || expected.cy != actual.cy)
            throw std::invalid_argument("native input already expects a different coverage camera");
    }
    if (config.lidar && !config.lidar->calibration_revision)
        throw std::invalid_argument("native LiDAR coverage requires an actual calibration revision");
    (void)MakeEvaluator(config);  // Uses the production visibility constructors' calibration validation.
}

/** @brief Compare the configured installation transform with real world poses. */
bool SameInstallation(const perception::AcquisitionLocalization& proof,
                      const perception::RigidTransform& base_from_sensor) {
    if (!proof.world_from_base.Valid() || !proof.world_from_sensor.Valid()) return false;
    if ((proof.world_from_base.Apply(base_from_sensor.translation) -
         proof.world_from_sensor.translation).Norm() > 1e-6) return false;
    for (const auto axis : {perception::Vec3{1, 0, 0}, perception::Vec3{0, 1, 0}, perception::Vec3{0, 0, 1}})
        if ((proof.world_from_base.Rotate(base_from_sensor.Rotate(axis)) -
             proof.world_from_sensor.Rotate(axis)).Norm() > 1e-6) return false;
    return true;
}

/** @brief Validate actual operation availability without asserting operation success. */
void ValidateMappingServices(const ExplorationServices& services, const ExplorationSessionOptions& options) {
    if (!services.request_map_refresh || !services.poll_map_refresh || !services.cancel_mapping ||
        ((options.require_save || options.save_on_cancel) && (!services.save_map || !services.poll_map_save)))
        throw std::invalid_argument("native exploration requires real versioned fusion and persistence operations");
    if (services.submit_navigation || services.poll_navigation || services.cancel_navigation || services.update_route_map)
        throw std::invalid_argument("mapping services cannot replace guarded navigation or own another route graph");
}

}  // namespace

bool ConfigureNativeExploration(const std::shared_ptr<ExplorationTask>& task,
    const std::shared_ptr<autolink::Node>& node, NativeExplorationConfiguration config,
    NativeExplorationPlatform platform) {
    if (!task || !node) throw std::invalid_argument("native exploration requires its real task and node");
    Validate(config, platform);
    if (config.camera) config.input.expected_depth_camera = config.camera->camera;
    // Native calibration callbacks enforce the canonical coverage intrinsics.
    perception::ValidateNativeObservationSourceOptions(config.input);
    auto settings = std::make_shared<const NativeExplorationConfiguration>(std::move(config));
    std::weak_ptr<ExplorationTask> weak_task = task;
    ExplorationTaskBindings bindings;
    bindings.options = settings->session;
    bindings.options.driver = ExplorationDriver::kBehaviorTree;
    bindings.prepare_mapping = platform.prepare_mapping;
    bindings.clock = platform.clock;
    bindings.process_sensor_packet = std::move(platform.process_sensor_packet);
    bindings.create_session = [node, weak_task, settings,
        create_mapping = platform.create_mapping_services,
        route_planner = platform.route_planner](
        const proto::ExplorationGoal& goal, const ExplorationSessionOptions& options) {
        const auto owner = weak_task.lock();
        if (!owner || options.mode != settings->session.mode || options.driver != ExplorationDriver::kBehaviorTree)
            throw std::logic_error("native exploration factory has a different owner or mode");
        // NavigationServices proves this callback runs on the current task factory owner.
        auto native_navigation = owner->NavigationServices(settings->routes.frame_id,
            settings->routes.planner_id, settings->controller_id,
            settings->routes.navigation_tree);
        auto services = create_mapping(goal, options);
        ValidateMappingServices(services, options);
        if (route_planner) {
            services.update_route_map = [route_planner](
                const perception::MapReadView& view) {
                route_planner->Update(view);
            };
        }
        // Keep an admitted observation goal alive across unrelated map
        // revisions, but revalidate its exact execution footprint. Coverage
        // updates must not preempt the controller during its first inference,
        // while newly occupied destinations still revoke navigation.
        services.navigation_still_valid = [coverage = settings->coverage](
            const AdaptedObservationGoal& goal, const Pose2d&, const MapReadView& map) {
            if (!map.grid || goal.intent.map_epoch != map.map_epoch) return false;
            auto safety = coverage.safety;
            if (map.footprint_inflated) {
                safety.robot_radius = 1e-9;
                safety.horizontal_margin = 0;
            }
            if (map.startup_support) safety.startup_support = map.startup_support;
            const autonomy::navigation::GeometrySafety validator(map.grid, safety);
            if (!validator.VerifyPose(goal.execution_pose).safe) return false;
            for (double yaw : goal.intent.required_views) {
                auto view = goal.execution_pose;
                view.yaw = yaw;
                if (!validator.VerifyPose(view).safe) return false;
            }
            return true;
        };
        services.submit_navigation = std::move(native_navigation.submit_navigation);
        services.poll_navigation = std::move(native_navigation.poll_navigation);
        services.cancel_navigation = std::move(native_navigation.cancel_navigation);
        std::shared_ptr<PathProvider> paths;
        if (route_planner) {
            paths = std::make_shared<VistaPathProvider>(route_planner);
        } else {
            paths = std::make_shared<NativeVistaPathProvider>(
                node, settings->routes);
        }
        auto evaluator = MakeEvaluator(*settings);
        auto coverage = settings->coverage;
        coverage.sampling.nominal_speed = options.nominal_speed;
        coverage.sampling.angular_speed = options.nominal_angular_speed;
        coverage.sampling.observation_time = options.minimum_observation_time;
        auto strategy = std::make_shared<HierarchicalCoverageStrategy>(paths, evaluator, coverage);
        auto adapter = std::make_shared<GoalPoseAdapter>(paths, evaluator, coverage.safety);
        return std::make_shared<ExplorationSession>(strategy, adapter, paths, std::move(services), options);
    };
    auto localization = [lookup = platform.localization, settings](perception::SensorKind sensor,
        const std::string& frame, double stamp) -> std::optional<perception::AcquisitionLocalization> {
        auto proof = lookup(sensor, frame, stamp);
        if (!proof || proof->world_frame != settings->routes.frame_id) return {};
        if (sensor == perception::SensorKind::kRgbd) {
            if (!settings->camera || proof->calibration_revision != settings->input.depth_calibration_revision ||
                !SameInstallation(*proof, settings->camera->base_from_camera)) return {};
        } else if (sensor == perception::SensorKind::kLidar) {
            if (!settings->lidar || proof->calibration_revision != settings->lidar->calibration_revision ||
                !SameInstallation(*proof, settings->lidar->base_from_lidar)) return {};
        } else return {};
        return proof;
    };
    return task->ConfigureNativeRuntime(task, node, std::move(bindings), settings->input,
                                       std::move(localization), settings->retirement_timeout);
}

}  // namespace autonomy::task::exploration
