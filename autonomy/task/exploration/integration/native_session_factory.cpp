/* Copyright 2026 The Openbot Authors. */
/** @file native_session_factory.cpp
 * @brief Owner-executed assembly and acquisition/coverage calibration
 * consistency.
 */
#include "autonomy/task/exploration/integration/native_session_factory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "autonomy/task/exploration/integration/task_domain_path_provider.hpp"
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"
#include "autonomy/task/exploration/observation/camera_visibility_model.hpp"
#include "autonomy/task/exploration/observation/lidar_visibility_model.hpp"
#include "autonomy/task/exploration/polygon_boundary.hpp"
#include "autonomy/task/exploration/routing/route_line_of_sight_view_producer.hpp"

namespace autonomy::task::exploration {
namespace {

/** @brief Build only visibility models supported by the configured physical
 * sensors. */
std::shared_ptr<ViewpointEvaluator> MakeEvaluator(
    const NativeExplorationConfiguration& config) {
    std::vector<std::shared_ptr<const SensorVisibilityModel>> models;
    if (config.camera)
        models.push_back(std::make_shared<CameraVisibilityModel>(
            config.camera->camera, config.camera->base_from_camera));
    if (config.lidar)
        models.push_back(std::make_shared<LidarVisibilityModel>(
            config.lidar->beam_directions, config.lidar->angular_tolerance,
            config.input.decoding.minimum_range,
            config.input.decoding.maximum_range,
            config.lidar->base_from_lidar));
    return std::make_shared<ViewpointEvaluator>(std::move(models));
}

}  // namespace

void ApplySharedExplorationOptions(
    const ::autonomy::perception::exploration::proto::ExplorationOptions&
        source,
    ExplorationSessionOptions* destination,
    HierarchicalCoverageOptions* coverage) {
    if (!destination)
        throw std::invalid_argument(
            "shared exploration options need a destination");
    if (source.min_frontier_cluster_size() <= 0) {
        throw std::invalid_argument("frontier cluster size must be positive");
    }
    if (source.has_frontier_cluster_tolerance_m() &&
        (!std::isfinite(source.frontier_cluster_tolerance_m()) ||
         source.frontier_cluster_tolerance_m() <= 0)) {
        throw std::invalid_argument(
            "frontier cluster tolerance must be finite and positive");
    }
    if (source.rush_home()) {
        throw std::invalid_argument(
            "rush_home cannot bypass Vista-owned return route geometry");
    }
    if (!source.boundary_ply_path().empty() ||
        !source.boundary_vgh_path().empty() ||
        !source.nogo_ply_path().empty()) {
        throw std::invalid_argument(
            "file-based polygon boundaries require runtime task geometry");
    }
    for (double value :
         {source.keypose_spacing_m(), source.lookahead_distance_m(),
          source.at_home_dist_m()}) {
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument(
                "shared keypose, lookahead, and home distances must be "
                "positive");
    }
    destination->keypose_spacing_m = source.keypose_spacing_m();
    destination->minimum_frontier_cluster_size =
        static_cast<std::size_t>(source.min_frontier_cluster_size());
    destination->frontier_cluster_tolerance_m =
        source.has_frontier_cluster_tolerance_m()
            ? source.frontier_cluster_tolerance_m()
            : 1.0;
    destination->use_frontier_points = source.use_frontier_points();
    // keypose_connect_dist_m is intentionally ignored: acquisition keyposes own
    // no edges and never query a route graph.
    destination->lookahead_distance_m = source.lookahead_distance_m();
    destination->return_position_tolerance = source.at_home_dist_m();
    destination->use_line_of_sight_lookahead =
        source.use_line_of_sight_lookahead();
    if (!coverage)
        return;
    if ((source.has_minimum_incremental_surface_gain() &&
         (!std::isfinite(source.minimum_incremental_surface_gain()) ||
          source.minimum_incremental_surface_gain() < 0 ||
          source.minimum_incremental_surface_gain() > 1)) ||
        (source.has_minimum_incremental_frontier_gain() &&
         (!std::isfinite(source.minimum_incremental_frontier_gain()) ||
          source.minimum_incremental_frontier_gain() < 0 ||
          source.minimum_incremental_frontier_gain() > 1)) ||
        (source.has_local_diversification_ratio() &&
         (!std::isfinite(source.local_diversification_ratio()) ||
          source.local_diversification_ratio() <= 0 ||
          source.local_diversification_ratio() > 1))) {
        throw std::invalid_argument("invalid shared local selection policy");
    }
    if (!std::isfinite(source.grid_cell_size_m()) ||
        source.grid_cell_size_m() <= 0 || source.grid_nearby_radius() <= 0 ||
        !std::isfinite(source.viewpoint_resolution_m()) ||
        source.viewpoint_resolution_m() <= 0 ||
        source.viewpoint_yaw_samples() <= 0 ||
        source.viewpoint_yaw_samples() > 360 ||
        !std::isfinite(source.collision_radius_m()) ||
        source.collision_radius_m() <= 0 || source.tsp_exact_max_nodes() <= 0 ||
        source.tsp_exact_max_nodes() > 18 ||
        source.local_path_optimization_iters() < 0 ||
        source.local_path_optimization_iters() > 64 ||
        source.momentum_thred() < 0 || source.far_path_momentum_thred() < 0 ||
        (source.use_momentum() && (!std::isfinite(source.momentum_dist_m()) ||
                                   source.momentum_dist_m() <= 0))) {
        throw std::invalid_argument(
            "invalid shared regional exploration options");
    }
    coverage->region_size = source.grid_cell_size_m();
    coverage->local_radius =
        source.grid_cell_size_m() * source.grid_nearby_radius();
    coverage->sampling.spacing = source.viewpoint_resolution_m();
    coverage->sampling.yaw_samples =
        static_cast<unsigned>(source.viewpoint_yaw_samples());
    if (!source.use_line_of_sight_lookahead() && !source.use_terrain_height())
        coverage->rolling_field.reset();
    coverage->safety.robot_radius = source.collision_radius_m();
    coverage->maximum_global_views = 128;
    coverage->route_optimization.maximum_nodes = 128;
    coverage->route_optimization.exact_maximum_nodes =
        static_cast<std::size_t>(source.tsp_exact_max_nodes());
    coverage->route_optimization.use_two_opt = source.tsp_use_two_opt();
    coverage->local_selection_attempts = std::max<std::size_t>(
        1, static_cast<std::size_t>(source.local_path_optimization_iters()));
    // These normalized ratios adapt upstream point-count admission knobs to the
    // same CoverageFusion gain semantics used by local candidate selection.
    if (source.has_minimum_incremental_surface_gain()) {
        coverage->minimum_incremental_surface_gain =
            source.minimum_incremental_surface_gain();
    }
    if (source.has_minimum_incremental_frontier_gain()) {
        coverage->minimum_incremental_frontier_gain =
            source.minimum_incremental_frontier_gain();
    }
    if (source.has_local_diversification_ratio()) {
        coverage->local_diversification_ratio =
            source.local_diversification_ratio();
    }
    if (source.has_local_diversification_seed()) {
        coverage->local_diversification_seed =
            source.local_diversification_seed();
    }
    coverage->use_frontier_points = source.use_frontier_points();
    coverage->use_momentum = source.use_momentum();
    coverage->momentum.sampling_distance = source.momentum_dist_m();
    coverage->momentum.activation_count_threshold =
        static_cast<unsigned>(source.momentum_thred());
    coverage->momentum.release_count_threshold =
        static_cast<unsigned>(source.far_path_momentum_thred());
}

namespace {

/** @brief Reject a split mode/calibration before native callbacks or factories
 * can run. */
void Validate(const NativeExplorationConfiguration& config,
              const NativeExplorationPlatform& platform) {
    perception::ValidateNativeObservationSourceOptions(config.input);
    const auto mode = config.session.mode;
    const bool depth = (perception::Mask(mode) &
                        perception::Mask(perception::SensorKind::kRgbd)) != 0;
    const bool lidar = (perception::Mask(mode) &
                        perception::Mask(perception::SensorKind::kLidar)) != 0;
    if (config.shared_options && config.shared_options->use_terrain_height()) {
        if (!depth || lidar) {
            throw std::invalid_argument(
                "terrain-height clearance requires the measured native RGB-D "
                "volume source");
        }
        if (!config.coverage.rolling_field) {
            throw std::invalid_argument(
                "terrain-height clearance requires a bounded measured 3-D "
                "field");
        }
        const auto& body = config.coverage.safety;
        if (!std::isfinite(body.robot_radius) || body.robot_radius <= 0 ||
            !std::isfinite(body.horizontal_margin) ||
            body.horizontal_margin < 0 || !std::isfinite(body.robot_height) ||
            body.robot_height <= 0 || !std::isfinite(body.vertical_margin) ||
            body.vertical_margin < 0 ||
            !std::isfinite(config.coverage.sampling.goal_clearance_reserve) ||
            config.coverage.sampling.goal_clearance_reserve < 0) {
            throw std::invalid_argument(
                "terrain-height clearance requires explicit finite 3-D body "
                "collision geometry");
        }
    }
    if (config.input.mode != mode || config.coverage.mode != mode ||
        config.session.use_frontier_points !=
            config.coverage.use_frontier_points ||
        !std::isfinite(config.coverage.minimum_incremental_surface_gain) ||
        config.coverage.minimum_incremental_surface_gain < 0 ||
        config.coverage.minimum_incremental_surface_gain > 1 ||
        !std::isfinite(config.coverage.minimum_incremental_frontier_gain) ||
        config.coverage.minimum_incremental_frontier_gain < 0 ||
        config.coverage.minimum_incremental_frontier_gain > 1 ||
        !std::isfinite(config.coverage.local_diversification_ratio) ||
        config.coverage.local_diversification_ratio <= 0 ||
        config.coverage.local_diversification_ratio > 1 ||
        config.camera.has_value() != depth ||
        config.lidar.has_value() != lidar ||
        config.session.validated_controller_modes.empty() ||
        !platform.create_mapping_services || !platform.localization ||
        !platform.clock ||
        (config.session.use_line_of_sight_lookahead &&
         !config.coverage.rolling_field) ||
        config.retirement_timeout <= std::chrono::milliseconds::zero() ||
        config.retirement_timeout > std::chrono::seconds(30))
        throw std::invalid_argument(
            "native exploration requires consistent modes and actual platform "
            "services");
    const auto required = perception::Mask(mode);
    bool mode_validated = false;
    for (const auto supported : config.session.validated_controller_modes) {
        if (supported == 0 ||
            supported > perception::Mask(perception::SensorMode::kLidarRgbd))
            throw std::invalid_argument("unsupported execution sensor mask");
        if (supported == required)
            mode_validated = true;
    }
    if (!mode_validated)
        throw std::invalid_argument(
            "selected sensing mode has no validated native execution mode");
    if (config.routes.service_name.empty() ||
        config.routes.planner_id.empty() ||
        config.routes.planner_id.size() > 256 ||
        config.routes.frame_id.empty() || config.routes.timeout.count() <= 0 ||
        config.routes.timeout > std::chrono::seconds(5) ||
        config.routes.retry_interval.count() < 0 ||
        config.routes.retry_interval > std::chrono::seconds(5) ||
        config.routes.maximum_poses < 2 ||
        config.routes.maximum_poses > 2000000)
        throw std::invalid_argument(
            "invalid native exploration route endpoint or budget");
    if (config.camera &&
        (config.camera->camera.min_depth != config.input.minimum_depth ||
         config.camera->camera.max_depth != config.input.maximum_depth))
        throw std::invalid_argument(
            "coverage camera and native depth validity ranges differ");
    if (config.camera && config.input.expected_depth_camera) {
        const auto& expected = *config.input.expected_depth_camera;
        const auto& actual = config.camera->camera;
        if (expected.width != actual.width ||
            expected.height != actual.height || expected.fx != actual.fx ||
            expected.fy != actual.fy || expected.cx != actual.cx ||
            expected.cy != actual.cy)
            throw std::invalid_argument(
                "native input already expects a different coverage camera");
    }
    if (config.lidar && !config.lidar->calibration_revision)
        throw std::invalid_argument(
            "native LiDAR coverage requires an actual calibration revision");
    (void)MakeEvaluator(config);  // Uses the production visibility
                                  // constructors' calibration validation.
}

/** @brief Compare the configured installation transform with real world poses.
 */
bool SameInstallation(const perception::AcquisitionLocalization& proof,
                      const perception::RigidTransform& base_from_sensor) {
    if (!proof.world_from_base.Valid() || !proof.world_from_sensor.Valid())
        return false;
    if ((proof.world_from_base.Apply(base_from_sensor.translation) -
         proof.world_from_sensor.translation)
            .Norm() > 1e-6)
        return false;
    for (const auto axis :
         {perception::Vec3{1, 0, 0}, perception::Vec3{0, 1, 0},
          perception::Vec3{0, 0, 1}})
        if ((proof.world_from_base.Rotate(base_from_sensor.Rotate(axis)) -
             proof.world_from_sensor.Rotate(axis))
                .Norm() > 1e-6)
            return false;
    return true;
}

/** @brief Validate actual operation availability without asserting operation
 * success. */
void ValidateMappingServices(const ExplorationServices& services,
                             const ExplorationSessionOptions& options) {
    if (!services.request_map_refresh || !services.poll_map_refresh ||
        !services.cancel_mapping ||
        ((options.require_save || options.save_on_cancel) &&
         (!services.save_map || !services.poll_map_save)))
        throw std::invalid_argument(
            "native exploration requires real versioned fusion and persistence "
            "operations");
    if (services.submit_navigation || services.poll_navigation ||
        services.cancel_navigation || services.update_route_map)
        throw std::invalid_argument(
            "mapping services cannot replace guarded navigation or own another "
            "route graph");
}

}  // namespace

bool ConfigureNativeExploration(const std::shared_ptr<ExplorationTask>& task,
                                const std::shared_ptr<autolink::Node>& node,
                                NativeExplorationConfiguration config,
                                NativeExplorationPlatform platform) {
    if (!task || !node)
        throw std::invalid_argument(
            "native exploration requires its real task and node");
    if (config.shared_options)
        ApplySharedExplorationOptions(*config.shared_options, &config.session,
                                      &config.coverage);
    Validate(config, platform);
    if (config.camera)
        config.input.expected_depth_camera = config.camera->camera;
    // Native calibration callbacks enforce the canonical coverage intrinsics.
    perception::ValidateNativeObservationSourceOptions(config.input);
    auto settings = std::make_shared<const NativeExplorationConfiguration>(
        std::move(config));
    std::weak_ptr<ExplorationTask> weak_task = task;
    ExplorationTaskBindings bindings;
    bindings.options = settings->session;
    bindings.options.driver = ExplorationDriver::kBehaviorTree;
    bindings.prepare_mapping = platform.prepare_mapping;
    bindings.clock = platform.clock;
    bindings.process_sensor_packet = std::move(platform.process_sensor_packet);
    bindings.create_session = [node, weak_task, settings,
                               create_mapping =
                                   platform.create_mapping_services,
                               route_planner = platform.route_planner](
                                  const proto::ExplorationGoal& goal,
                                  const ExplorationSessionOptions& options) {
        const auto owner = weak_task.lock();
        if (!owner || options.mode != settings->session.mode ||
            options.driver != ExplorationDriver::kBehaviorTree)
            throw std::logic_error(
                "native exploration factory has a different owner or mode");
        // NavigationServices proves this callback runs on the current task
        // factory owner.
        auto native_navigation = owner->NavigationServices(
            settings->routes.frame_id, settings->routes.planner_id,
            settings->controller_id, settings->routes.navigation_tree);
        auto services = create_mapping(goal, options);
        ValidateMappingServices(services, options);
        if (route_planner) {
            services.update_route_map =
                [route_planner](const perception::MapReadView& view) {
                    route_planner->Update(view);
                };
        }
        // Keep an admitted observation goal alive across unrelated map
        // revisions, but revalidate its exact execution footprint. Coverage
        // updates must not preempt the controller during its first inference,
        // while newly occupied destinations still revoke navigation.
        services.navigation_still_valid =
            [coverage = settings->coverage,
             allowed_region = options.allowed_region,
             allowed_polygon = options.allowed_polygon,
             no_go_areas = options.no_go_areas](
                const AdaptedObservationGoal& goal, const Pose2d&,
                const MapReadView& map) {
                if (!map.grid || goal.intent.map_epoch != map.map_epoch ||
                    (allowed_region &&
                     !allowed_region->Contains(goal.execution_pose)) ||
                    (allowed_polygon &&
                     !PolygonBoundaryContains(*allowed_polygon,
                                              goal.execution_pose.x,
                                              goal.execution_pose.y)) ||
                    !PolygonBoundaryOutsideAll(no_go_areas,
                                               goal.execution_pose.x,
                                               goal.execution_pose.y)) {
                    return false;
                }
                auto safety = coverage.safety;
                if (map.footprint_inflated) {
                    safety.robot_radius = 1e-9;
                    safety.horizontal_margin = 0;
                }
                if (map.startup_support)
                    safety.startup_support = map.startup_support;
                const autonomy::navigation::GeometrySafety validator(map.grid,
                                                                     safety);
                if (!validator.VerifyPose(goal.execution_pose).safe)
                    return false;
                for (double yaw : goal.intent.required_views) {
                    auto view = goal.execution_pose;
                    view.yaw = yaw;
                    if (!validator.VerifyPose(view).safe)
                        return false;
                }
                return true;
            };
        services.submit_navigation =
            std::move(native_navigation.submit_navigation);
        services.poll_navigation = std::move(native_navigation.poll_navigation);
        services.cancel_navigation =
            std::move(native_navigation.cancel_navigation);
        std::shared_ptr<PathProvider> paths;
        if (route_planner) {
            paths = std::make_shared<VistaPathProvider>(route_planner);
        } else {
            paths = std::make_shared<NativeVistaPathProvider>(node,
                                                              settings->routes);
        }
        paths = std::make_shared<TaskDomainPathProvider>(
            std::move(paths), options.allowed_region, options.allowed_polygon,
            options.no_go_areas);
        auto evaluator = MakeEvaluator(*settings);
        auto coverage = settings->coverage;
        coverage.sampling.nominal_speed = options.nominal_speed;
        coverage.sampling.angular_speed = options.nominal_angular_speed;
        coverage.sampling.observation_time = options.minimum_observation_time;
        coverage.sampling.allowed_region = options.allowed_region;
        coverage.sampling.allowed_polygon = options.allowed_polygon;
        coverage.sampling.no_go_areas = options.no_go_areas;
        auto strategy = std::make_shared<HierarchicalCoverageStrategy>(
            paths, evaluator, coverage);
        if (options.use_line_of_sight_lookahead) {
            auto producer =
                std::make_shared<const RouteLineOfSightViewProducer>(
                    RouteLineOfSightViewProducerOptions{
                        coverage.safety,
                        coverage.sampling.goal_clearance_reserve});
            std::weak_ptr<HierarchicalCoverageStrategy> weak_strategy =
                strategy;
            services.route_line_of_sight_view =
                [weak_strategy, producer](
                    const MapReadView& map,
                    const std::function<bool()>& canceled) {
                    const auto owner = weak_strategy.lock();
                    if (!owner)
                        return std::optional<RouteLineOfSightView>{};
                    return producer->Produce(map, owner->RollingFieldView(),
                                             canceled);
                };
        }
        auto adapter = std::make_shared<GoalPoseAdapter>(
            paths, evaluator, coverage.safety,
            coverage.sampling.goal_clearance_reserve);
        return std::make_shared<ExplorationSession>(
            strategy, adapter, paths, std::move(services), options);
    };
    auto localization = [lookup = platform.localization, settings](
                            perception::SensorKind sensor,
                            const std::string& frame, double stamp)
        -> std::optional<perception::AcquisitionLocalization> {
        auto proof = lookup(sensor, frame, stamp);
        if (!proof || proof->world_frame != settings->routes.frame_id)
            return {};
        if (sensor == perception::SensorKind::kRgbd) {
            if (!settings->camera ||
                proof->calibration_revision !=
                    settings->input.depth_calibration_revision ||
                !SameInstallation(*proof, settings->camera->base_from_camera))
                return {};
        } else if (sensor == perception::SensorKind::kLidar) {
            if (!settings->lidar ||
                proof->calibration_revision !=
                    settings->lidar->calibration_revision ||
                !SameInstallation(*proof, settings->lidar->base_from_lidar))
                return {};
        } else
            return {};
        return proof;
    };
    return task->ConfigureNativeRuntime(
        task, node, std::move(bindings), settings->input,
        std::move(localization), settings->retirement_timeout);
}

}  // namespace autonomy::task::exploration
