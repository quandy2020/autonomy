/* Copyright 2026 The Openbot Authors. */
/** @file native_session_factory_test.cpp
 * @brief Actual task-owner, native source and shared graph assembly
 * regressions.
 * @details Generated sensor messages travel through real Autolink Readers. The
 * localization and mapping owners are controlled dependencies: these tests
 * prove assembly and rejection behavior, not SLAM fusion or physical robot
 * exploration.
 */
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "autolink/autolink.hpp"
#include "autonomy/perception/observation/native_sensor_messages.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/task/exploration/integration/native_session_factory.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace t = autonomy::task;
namespace p = autonomy::perception;
namespace v = autonomy::planning::planner::vista;
namespace m = automsgs::msgs::sensor_msgs;
namespace n = autonomy::navigation;
using namespace std::chrono_literals;
std::atomic<bool> cleanup_failed{false};  ///< A failed native Reader drain must
                                          ///< affect the final test result.

/** @brief Fail a concrete assembly, provenance or transport invariant. */
void Require(bool value, const char* reason) {
    if (!value)
        throw std::runtime_error(reason);
}
/** @brief Await an actual transition under a finite steady-clock deadline. */
template <typename Predicate>
void Await(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!predicate()) {
        Require(std::chrono::steady_clock::now() < deadline, reason);
        std::this_thread::sleep_for(1ms);
    }
}

/** @brief Exact rectified camera and optical installation used by native input.
 */
e::NativeCameraCoverage Camera() {
    e::NativeCameraCoverage result;
    result.camera = {2, 2, 2, 2, .5, .5, .1, 8};
    result.base_from_camera.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    result.base_from_camera.translation.z = .5;
    return result;
}
/** @brief Fixed three-mode settings with distinct real sensor channels. */
e::NativeExplorationConfiguration Configuration(p::SensorMode mode,
                                                const std::string& suffix) {
    e::NativeExplorationConfiguration config;
    config.session.mode = config.coverage.mode = config.input.mode = mode;
    config.session.validated_controller_modes = {p::Mask(mode)};
    config.session.health.recovery_hold = 0;
    config.session.pose_max_age = config.session.map_max_age = 10;
    config.coverage.safety.robot_radius = .1;
    config.coverage.safety.horizontal_margin = 0;
    config.input.depth_topic =
        "/autonomy_test/native_factory/" + suffix + "/depth";
    config.input.camera_info_topic =
        "/autonomy_test/native_factory/" + suffix + "/info";
    config.input.lidar_topic =
        "/autonomy_test/native_factory/" + suffix + "/points";
    config.input.depth_calibration_revision = 7;
    config.input.depth_rectified = true;
    config.input.decoding.instantaneous_cloud = true;
    config.input.decoding.expected_scan_points = 1;
    if (p::Mask(mode) & p::Mask(p::SensorKind::kRgbd))
        config.camera = Camera();
    if (p::Mask(mode) & p::Mask(p::SensorKind::kLidar)) {
        e::NativeLidarCoverage lidar;
        lidar.beam_directions = {{1, 0, 0}};
        lidar.base_from_lidar.translation.z = .5;
        lidar.calibration_revision = 7;
        config.lidar = lidar;
    }
    return config;
}

/** @brief Controlled actual provider counters remain alive through task
 * cleanup. */
struct PlatformState {
    std::atomic<unsigned> factories{0}, refreshes{0},
        saves{0};  ///< Real owner callback invocations, not successful
                   ///< operations.
    std::atomic<double> now{3.4};  ///< Controlled monotonic task time, advanced
                                   ///< only for explicit budget transitions.
    std::atomic<bool> wrong_owner{
        false};  ///< Detects assembly on the test/caller thread.
    const std::thread::id caller =
        std::this_thread::get_id();  ///< Thread that configures the runtime.
    bool incorrect_installation =
        false;  ///< Deliberate actual pose/coverage calibration disagreement.
    std::atomic<bool> missing_fusion{
        false};  ///< Deliberate mapper with no fusion operation.
    std::atomic<bool> missing_save{false};  ///< Deliberate mapper without the
                                            ///< required persistence operation.
    std::atomic<unsigned> navigations{
        0};  ///< Actual guarded submissions, which assembly failure must never
             ///< issue.
};

/** @brief Provide acquisition poses and deliberately withhold fusion/save
 * receipts. */
e::NativeExplorationPlatform Platform(
    const std::shared_ptr<PlatformState>& state) {
    e::NativeExplorationPlatform platform;
    platform.clock = [state] { return state->now.load(); };
    platform.localization = [state](p::SensorKind sensor,
                                    const std::string& frame, double stamp) {
        p::AcquisitionLocalization proof;
        proof.world_frame = "map";
        proof.sensor_frame = frame;
        proof.world_from_base.translation = {1, 1, 0};
        if (sensor == p::SensorKind::kRgbd)
            proof.world_from_sensor.rotation =
                Camera().base_from_camera.rotation;
        proof.world_from_sensor.translation = {
            1, 1, state->incorrect_installation ? .8 : .5};
        proof.calibration_revision = 7;
        proof.map_epoch = 4;
        proof.support = p::Mask(p::SensorMode::kLidarRgbd);
        proof.healthy = true;
        proof.valid_from = stamp - 1;
        proof.valid_until = stamp + 1;
        return std::optional<p::AcquisitionLocalization>{proof};
    };
    platform.create_mapping_services =
        [state](const t::proto::ExplorationGoal&,
                const e::ExplorationSessionOptions&) {
            if (std::this_thread::get_id() == state->caller)
                state->wrong_owner = true;
            ++state->factories;
            e::ExplorationServices services;
            if (!state->missing_fusion)
                services.request_map_refresh =
                    [state](const e::RequestVersion&) {
                        ++state->refreshes;
                        return true;
                    };
            services.poll_map_refresh = [] {
                return std::optional<e::MappingReply>{};
            };
            if (!state->missing_save)
                services.save_map = [state](const std::string&,
                                            const e::RequestVersion&) {
                    ++state->saves;
                    return true;
                };
            services.poll_map_save = [] {
                return std::optional<e::MappingReply>{};
            };
            services.cancel_mapping = [](const e::RequestVersion&) {};
            return services;
        };
    return platform;
}

/** @brief Actual task/native readers, drained before their shared dependencies
 * die. */
struct TaskOwner {
    std::shared_ptr<PlatformState> state =
        std::make_shared<PlatformState>();  ///< Independently controlled
                                            ///< external dependencies.
    std::shared_ptr<autolink::Node> node;  ///< Real native communication owner.
    std::shared_ptr<t::ExplorationTask> task =
        std::make_shared<t::ExplorationTask>();  ///< Production owner/BT loop.
    e::NativeExplorationConfiguration
        config;  ///< Exact immutable assembly settings.
    TaskOwner(p::SensorMode mode, const std::string& name)
        : config(Configuration(mode, name)) {
        node = autolink::CreateNode("native_factory_" + name);
        Require(node != nullptr, "real task Node created");
        task->SetNode(node);
        task->SetGuardedSubmitNavigation(
            [evidence = state](const auto&, const auto& permit) {
                ++evidence->navigations;
                return permit && permit();
            });
        task->SetVersionedNavigationProbe(
            [](const auto&, auto*) { return false; },
            [](const auto&) { return true; });
    }
    /** @brief Install dependencies atomically and then start the production
     * owner. */
    void Initialize() {
        Require(
            e::ConfigureNativeExploration(task, node, config, Platform(state)),
            "native runtime installed atomically");
        t::proto::TaskServerOptions options;
        options.mutable_scheduler()->set_feedback_period_ms(2);
        Require(task->Initialize(options), "real task initialized");
        Await(
            [&] { return task->GetObservationSourceSnapshot().source.running; },
            "real native source started");
    }
    /** @brief Always verify actual source retirement, including assertion
     * failure paths. */
    ~TaskOwner() {
        task->Shutdown();
        const auto completion = task->ObservationSourceShutdownCompletion();
        if (completion.valid() &&
            (completion.wait_for(5s) != std::future_status::ready ||
             !completion.get()))
            cleanup_failed = true;
    }
};

/** @brief One real native camera calibration matching the configured coverage
 * model. */
m::CameraInfo Calibration(double focal = 2) {
    m::CameraInfo info;
    info.mutable_header()->set_frame_id("depth_optical");
    info.set_width(2);
    info.set_height(2);
    for (double value : {focal, 0., .5, 0., 2., .5, 0., 0., 1.})
        info.add_k(value);
    for (double value : {focal, 0., .5, 0., 0., 2., .5, 0., 0., 0., 1., 0.})
        info.add_p(value);
    return info;
}
/** @brief Encode finite float data explicitly in native little-endian storage.
 */
void Float(std::string& bytes, unsigned offset, float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i)
        bytes.at(offset + i) = static_cast<char>((bits >> (8 * i)) & 255);
}
/** @brief Preserve the actual controlled acquisition time in a generated native
 * Header. */
template <typename Message>
void AcquisitionStamp(Message& message, double seconds) {
    Require(std::isfinite(seconds) && seconds >= 0 && seconds < 1000000,
            "finite bounded acquisition timestamp");
    const auto whole = static_cast<std::int64_t>(std::floor(seconds));
    const auto nanos =
        static_cast<std::int64_t>(std::llround((seconds - whole) * 1000000000));
    auto* stamp = message.mutable_header()->mutable_stamp();
    stamp->set_sec(whole + nanos / 1000000000);
    stamp->set_nanosec(static_cast<std::uint32_t>(nanos % 1000000000));
}
/** @brief Stamped full rectified depth raster in real automsgs bytes. */
m::Image Depth(double stamp) {
    m::Image image;
    image.mutable_header()->set_frame_id("depth_optical");
    AcquisitionStamp(image, stamp);
    image.set_width(2);
    image.set_height(2);
    image.set_step(8);
    image.set_encoding("32FC1");
    std::string bytes(16, '\0');
    for (unsigned i = 0; i < 16; i += 4)
        Float(bytes, i, 1);
    image.set_data(bytes);
    return image;
}
/** @brief Stamped simultaneous cloud with an actual valid sensor-frame hit. */
m::PointCloud2 Cloud(double stamp) {
    m::PointCloud2 cloud;
    cloud.mutable_header()->set_frame_id("lidar");
    AcquisitionStamp(cloud, stamp);
    cloud.set_width(1);
    cloud.set_height(1);
    cloud.set_point_step(12);
    cloud.set_row_step(12);
    unsigned offset = 0;
    for (const char* name : {"x", "y", "z"}) {
        auto* field = cloud.add_fields();
        field->set_name(name);
        field->set_offset(offset);
        field->set_count(1);
        field->set_datatype(m::PointField::FLOAT32);
        offset += 4;
    }
    std::string bytes(12, '\0');
    Float(bytes, 0, 1);
    cloud.set_data(bytes);
    return cloud;
}

/** @brief Send only after the real writer has discovered the actual native
 * reader. */
template <typename Message>
void Publish(const std::shared_ptr<autolink::Writer<Message>>& writer,
             const Message& message) {
    Require(writer != nullptr, "real native Writer created");
    Await([&] { return writer->HasReader(); },
          "actual native Reader discovered");
    Require(writer->Write(message),
            "generated native message accepted by transport");
}
/** @brief Deliver calibrated measurements through real source subscriptions. */
void Observe(TaskOwner& owner, bool expect_rejection = false) {
    const auto mask = p::Mask(owner.config.session.mode);
    if (mask & p::Mask(p::SensorKind::kRgbd)) {
        auto info = owner.node->CreateWriter<m::CameraInfo>(
            owner.config.input.camera_info_topic);
        auto depth =
            owner.node->CreateWriter<m::Image>(owner.config.input.depth_topic);
        Publish(info, Calibration());
        Await(
            [&] {
                return owner.task->GetObservationSourceSnapshot()
                    .source.depth_calibration_ready;
            },
            "actual matching CameraInfo processed before the depth frame");
        const auto before = owner.task->GetObservationSourceSnapshot().source;
        Publish(depth, Depth(owner.state->now.load()));
        Await(
            [&] {
                const auto after =
                    owner.task->GetObservationSourceSnapshot().source;
                return expect_rejection
                           ? after.rejected[1] > before.rejected[1]
                           : after.accepted[1] > before.accepted[1];
            },
            "new actual depth packet admitted or deliberately rejected");
    }
    if (mask & p::Mask(p::SensorKind::kLidar)) {
        auto cloud = owner.node->CreateWriter<m::PointCloud2>(
            owner.config.input.lidar_topic);
        const auto before =
            owner.task->GetObservationSourceSnapshot().source.accepted[0];
        Publish(cloud, Cloud(owner.state->now.load()));
        Await(
            [&] {
                return owner.task->GetObservationSourceSnapshot()
                           .source.accepted[0] > before;
            },
            "actual cloud packet admitted");
    }
}

/** @brief Static inconsistent settings fail before either installed dependency
 * changes. */
void ConfigurationRejectionIsAtomic() {
    TaskOwner owner(p::SensorMode::kRgbdOnly, "atomic");
    const auto platform = Platform(owner.state);
    auto invalid = owner.config;
    invalid.input.camera_info_topic = invalid.input.depth_topic;
    bool rejected = false;
    try {
        (void)e::ConfigureNativeExploration(owner.task, owner.node, invalid,
                                            platform);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(
        rejected && !owner.task->GetObservationSourceSnapshot().configured &&
            owner.state->factories == 0,
        "invalid runtime did not partially install source or invoke factory");
    auto foreign = autolink::CreateNode("native_factory_wrong_node");
    Require(!e::ConfigureNativeExploration(owner.task, foreign, owner.config,
                                           platform) &&
                !owner.task->GetObservationSourceSnapshot().configured,
            "different task/source/route Node rejected without partial "
            "installation");
    invalid = owner.config;
    invalid.session.use_line_of_sight_lookahead = true;
    rejected = false;
    try {
        (void)e::ConfigureNativeExploration(owner.task, owner.node, invalid,
                                            platform);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected && !owner.task->GetObservationSourceSnapshot().configured,
            "LOS lookahead without the sole rolling field fails atomically");
    owner.Initialize();
    invalid = owner.config;
    invalid.session.validated_controller_modes = {
        p::Mask(p::SensorKind::kLidar)};
    rejected = false;
    try {
        (void)e::ConfigureNativeExploration(owner.task, owner.node, invalid,
                                            platform);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected &&
                owner.task->GetObservationSourceSnapshot().source.running &&
                owner.state->factories == 0,
            "unvalidated execution mode left the existing real source running");
}

/** @brief Real input cannot silently use a different coverage/installation
 * calibration. */
void CalibrationDisagreementIsRejected() {
    TaskOwner owner(p::SensorMode::kRgbdOnly, "installation");
    owner.state->incorrect_installation = true;
    owner.Initialize();
    Observe(owner, true);
    Require(
        owner.task->GetObservationSourceSnapshot().source.accepted[1] == 0 &&
            owner.state->factories == 0,
        "actual wrong installation cannot seed a session or count as usable "
        "sensing");
    auto info = owner.node->CreateWriter<m::CameraInfo>(
        owner.config.input.camera_info_topic);
    Publish(info, Calibration(3));
    Await(
        [&] {
            return owner.task->GetObservationSourceSnapshot()
                .source.calibration_invalidated;
        },
        "different native CameraInfo invalidated coverage calibration");
}

/** @brief An owner-side factory failure drains its dispatcher and admits a
 * corrected configuration. */
void MissingMappingOperationsFailBeforeMotion() {
    unsigned index = 0;
    for (const bool missing_save : {false, true}) {
        TaskOwner owner(p::SensorMode::kRgbdOnly,
                        "missing_mapping_" + std::to_string(index++));
        owner.state->missing_fusion = !missing_save;
        owner.state->missing_save = missing_save;
        owner.Initialize();
        Observe(owner);
        auto grid = std::make_shared<n::GridMap>(20, 20, .2);
        grid->Fill({n::CellState::kFree, 0});
        p::MapReadView view;
        view.grid = grid;
        view.frame_id = "map";
        view.map_epoch = 4;
        view.geometry_revision = 7;
        view.source_stamp = 3.25;
        Require(owner.task->PushMap(view),
                "actual input map admitted before failed assembly");
        t::proto::ExplorationGoal goal;
        goal.set_command(t::proto::EXPLORATION_CMD_START);
        goal.mutable_header()->set_task_id("missing-mapping-" +
                                           std::to_string(index));
        Require(owner.task->SubmitGoal(goal),
                "real task accepted generation before owner validates mapping "
                "services");
        Await(
            [&] {
                return owner.task->GetLifecycle() == t::TaskLifecycle::kFailed;
            },
            "missing fusion or required save operation fails on the actual "
            "owner");
        Await([&] { return owner.task->MotionDrained(); },
              "failed factory's real navigation dispatcher drained");
        Require(owner.state->factories == 1 && !owner.state->wrong_owner &&
                    owner.state->refreshes == 0 &&
                    owner.state->navigations == 0,
                "failed assembly made no fusion requests or native motion "
                "submissions");
        const auto snapshot = owner.task->GetExplorationSnapshot();
        Require(!snapshot || !snapshot->completion_confirmed,
                "failed assembly cannot certify exploration");
        owner.state->missing_fusion = false;
        owner.state->missing_save = false;
        Require(
            e::ConfigureNativeExploration(owner.task, owner.node, owner.config,
                                          Platform(owner.state)),
            "owner cleanup permits atomically installing corrected "
            "dependencies");
    }
}

/** @brief The real server owns the only Vista graph during three-mode owner
 * assembly. */
void OwnerFactoryAndSharedGraph() {
    autonomy::planning::proto::PlannerOptions options;
    options.add_planner_plugins("vista_planner");
    options.set_default_planner_id("vista_planner");
    options.mutable_vista()->set_horizontal_margin(0);
    options.mutable_vista()->set_maximum_nodes(8);
    auto* map = options.mutable_costmap();
    map->set_enabled(true);
    map->set_name("native_factory_map");
    map->set_frame_id("map");
    map->set_resolution(.2);
    map->set_width(4);
    map->set_height(4);
    map->set_robot_radius(.1);
    map->add_plugins("none");
    autonomy::planning::PlannerServer server(options);
    auto plugin = std::dynamic_pointer_cast<v::VistaPlanner>(
        server.GetPlanner("vista_planner"));
    Require(plugin != nullptr, "real PlannerServer owns its Vista plugin");
    const auto graph = plugin->SharedRoutePlanner();
    auto grid = std::make_shared<n::GridMap>(20, 20, .2);
    grid->Fill({n::CellState::kFree, 0});
    p::MapReadView view;
    view.grid = grid;
    view.frame_id = "map";
    view.map_epoch = 4;
    view.geometry_revision = 7;
    view.source_stamp = 3.25;
    plugin->SetMapViewSource(
        [view] { return std::optional<p::MapReadView>{view}; });
    unsigned index = 0;
    for (auto mode : {p::SensorMode::kRgbdOnly, p::SensorMode::kLidarOnly,
                      p::SensorMode::kLidarRgbd}) {
        TaskOwner owner(mode, "mode_" + std::to_string(index++));
        owner.config.session.time_budget = .1;
        owner.config.session.return_margin = 0;
        owner.Initialize();
        Require(owner.state->factories == 0,
                "configuration did not build or start a Session on the caller");
        Observe(owner);
        Require(owner.task->PushMap(view),
                "immutable native consumer map admitted");
        t::proto::ExplorationGoal goal;
        goal.set_command(t::proto::EXPLORATION_CMD_START);
        goal.mutable_header()->set_task_id("factory-mode-" +
                                           std::to_string(index));
        Require(owner.task->SubmitGoal(goal),
                "real task accepted this exploration generation");
        const auto identity = owner.task->SensorInputIdentity();
        Require(identity.has_value(),
                "accepted generation has a fixed input identity");
        Await(
            [&] {
                const auto source = owner.task->GetObservationSourceSnapshot();
                return source.identity &&
                       source.identity->generation == identity->generation &&
                       source.source.running;
            },
            "new generation's source replaced retired pre-start readers");
        Await(
            [&] {
                const auto snapshot = owner.task->GetExplorationSnapshot();
                return snapshot && snapshot->session_id != 0;
            },
            "real owner created and started the BT Session");
        Observe(owner);
        try {
            Await(
                [&] {
                    const auto state = owner.task->GetExplorationSnapshot();
                    return state && state->sensors.ready &&
                           state->sensors.active_sensors == p::Mask(mode);
                },
                "actual post-start measurements made the selected sensing mode "
                "ready");
        } catch (...) {
            const auto state = owner.task->GetExplorationSnapshot();
            const auto source = owner.task->GetObservationSourceSnapshot();
            std::cerr << "mode=" << unsigned(p::Mask(mode))
                      << " lifecycle=" << int(owner.task->GetLifecycle())
                      << " source_accepted=" << source.source.accepted[0] << ","
                      << source.source.accepted[1];
            if (state)
                std::cerr << " state=" << int(state->state) << " sensors="
                          << unsigned(state->sensors.active_sensors)
                          << " accepted="
                          << state->accepted_observations.sequence[0] << ","
                          << state->accepted_observations.sequence[1]
                          << " processed="
                          << state->processed_observations.sequence[0] << ","
                          << state->processed_observations.sequence[1]
                          << " losses="
                          << unsigned(state->observation_losses.pending_sources)
                          << " reason=" << state->reason;
            std::cerr << std::endl;
            throw;
        }
        // Refresh is a finish/save barrier, not an unconditional START side
        // effect. Exceed the explicit task budget while the real measured input
        // stays fresh.
        owner.state->now = 3.6;
        Await(
            [&] {
                const auto state = owner.task->GetExplorationSnapshot();
                return owner.state->refreshes > 0 && state &&
                       state->result.exploration ==
                           e::ExplorationOutcome::kPartialBudget;
            },
            "actual Session requested a fusion barrier and published its "
            "partial budget outcome");
        const auto snapshot = owner.task->GetExplorationSnapshot();
        Require(owner.state->factories == 1 && !owner.state->wrong_owner &&
                    snapshot &&
                    snapshot->result.exploration ==
                        e::ExplorationOutcome::kPartialBudget &&
                    !snapshot->completion_confirmed &&
                    !snapshot->result.Success(true, true) &&
                    owner.state->saves == 0,
                "budget exit preserves partial outcome and cannot save before "
                "the independent fusion receipt");
        Require(plugin->SharedRoutePlanner() == graph && !plugin->LastRoute(),
                "factory queries retain the server's only graph and "
                "navigation's route state");
        Require(
            !e::ConfigureNativeExploration(owner.task, owner.node, owner.config,
                                           Platform(owner.state)),
            "active Session cannot be partially reconfigured");
    }
}

/** @brief Generated settings map once without creating keypose graph policy. */
void SharedOptionsBridgeIsExact() {
    ::autonomy::perception::exploration::proto::ExplorationOptions source;
    source.set_keypose_spacing_m(2.5);
    source.set_keypose_connect_dist_m(99);
    source.set_lookahead_distance_m(4.25);
    source.set_at_home_dist_m(.6);
    source.set_use_line_of_sight_lookahead(true);
    source.set_viewpoint_num_x(40);
    source.set_viewpoint_num_y(30);
    source.set_viewpoint_num_z(10);
    source.set_grid_cell_size_m(6.0);
    source.set_grid_nearby_radius(5);
    source.set_min_frontier_cluster_size(9);
    source.set_viewpoint_resolution_m(.75);
    source.set_viewpoint_yaw_samples(12);
    source.set_collision_radius_m(.35);
    source.set_tsp_exact_max_nodes(12);
    source.set_tsp_use_two_opt(true);
    source.set_local_path_optimization_iters(7);
    source.set_use_frontier_points(false);
    source.set_use_momentum(true);
    source.set_momentum_dist_m(1.25);
    source.set_momentum_thred(3);
    source.set_far_path_momentum_thred(8);
    e::ExplorationSessionOptions destination;
    e::HierarchicalCoverageOptions coverage;
    e::RollingViewpointFieldOptions field;
    field.dimensions = {40, 30, 10};
    field.rollover_step = {10, 7, 2};
    field.maximum_cells = 12000;
    field.maximum_input_voxels = 12000;
    coverage.rolling_field = field;
    e::ApplySharedExplorationOptions(source, &destination, &coverage);
    Require(destination.keypose_spacing_m == 2.5 &&
                destination.lookahead_distance_m == 4.25 &&
                destination.return_position_tolerance == .6 &&
                destination.use_line_of_sight_lookahead &&
                destination.minimum_frontier_cluster_size == 9 &&
                coverage.region_size == 6.0 && coverage.local_radius == 30.0 &&
                coverage.sampling.spacing == .75 &&
                coverage.sampling.yaw_samples == 12 && coverage.rolling_field &&
                coverage.rolling_field->dimensions.isApprox(
                    Eigen::Vector3i(40, 30, 10)) &&
                coverage.rolling_field->maximum_cells == 12000 &&
                coverage.safety.robot_radius == .35 &&
                coverage.maximum_global_views == 128 &&
                coverage.route_optimization.maximum_nodes == 128 &&
                coverage.route_optimization.exact_maximum_nodes == 12 &&
                coverage.route_optimization.use_two_opt &&
                coverage.route_optimization.improvement_iterations == 16 &&
                coverage.local_selection_attempts == 7 &&
                !coverage.use_frontier_points && coverage.use_momentum &&
                coverage.momentum.sampling_distance == 1.25 &&
                coverage.momentum.activation_count_threshold == 3 &&
                coverage.momentum.release_count_threshold == 8,
            "generated native exploration policy maps exactly once");
    auto polygon = source;
    polygon.set_boundary_ply_path("mission-boundary.ply");
    bool polygon_rejected = false;
    try {
        e::ApplySharedExplorationOptions(polygon, &destination, &coverage);
    } catch (const std::invalid_argument&) {
        polygon_rejected = true;
    }
    Require(polygon_rejected,
            "file polygons cannot be silently reduced to rectangular bounds");
    source.set_keypose_spacing_m(0);
    bool rejected = false;
    try {
        e::ApplySharedExplorationOptions(source, &destination);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected,
            "invalid generated distances fail before native assembly");
}

}  // namespace

/** @brief Run only actual native assembly and calibration regressions. */
int main(int argc, char** argv) {
    if (argc < 1 || !autolink::Init(argv[0]))
        return 2;
    int failures = 0;
    for (const auto& item : std::vector<std::pair<const char*, void (*)()>>{
             {"shared_options_bridge_exact", SharedOptionsBridgeIsExact},
             {"configuration_atomic_rejection", ConfigurationRejectionIsAtomic},
             {"calibration_disagreement", CalibrationDisagreementIsRejected},
             {"missing_mapping_operations_cleanup",
              MissingMappingOperationsFailBeforeMotion},
             {"owner_factory_shared_graph_three_modes",
              OwnerFactoryAndSharedGraph}}) {
        try {
            item.second();
            std::cout << "PASS " << item.first << std::endl;
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << item.first << ": " << error.what()
                      << std::endl;
        }
    }
    autolink::Clear();
    return failures || cleanup_failed ? 1 : 0;
}
