/* Copyright 2026 The Openbot Authors. */
/** @file native_session_factory.hpp
 * @brief Native task/source/strategy assembly using the PlannerServer's shared graph.
 */
#pragma once

#include "autonomy/task/exploration/exploration.hpp"
#include "autonomy/task/exploration/integration/native_vista_path_provider.hpp"
#include "autonomy/task/exploration/strategy/hierarchical_coverage_strategy.hpp"

namespace autonomy::task::exploration {

/** @brief Pinhole coverage calibration of the actual rectified depth stream. */
struct NativeCameraCoverage {
    perception::CameraModel camera;  ///< Must match CameraInfo and native valid depth bounds.
    perception::RigidTransform base_from_camera;  ///< Actual optical-frame installation transform.
};

/** @brief Actual beam support and installation calibration for a 3D LiDAR. */
struct NativeLidarCoverage {
    std::vector<perception::Vec3> beam_directions;  ///< Unit rays in the sensor frame; no invented omnidirectional support.
    perception::RigidTransform base_from_lidar;  ///< Installation transform attested by acquisition localization.
    std::uint64_t calibration_revision = 0;  ///< Nonzero identity of these extrinsics and beam settings.
    double angular_tolerance = .02;  ///< Beam angular support in radians, strictly between zero and 1.57.
};

/** @brief One immutable configuration shared by native input and decision consumers. */
struct NativeExplorationConfiguration {
    ExplorationSessionOptions session;  ///< Explicit scope, budgets and independently validated execution modes.
    HierarchicalCoverageOptions coverage;  ///< Sampling, fair revisit and footprint policy.
    perception::NativeObservationSourceOptions input;  ///< Native topics, depth calibration, units and beam timing.
    NativeVistaPathOptions routes;  ///< Same planner id and frame used for native navigation dispatch.
    std::optional<NativeCameraCoverage> camera;  ///< Required exactly when RGB-D participates in this mode.
    std::optional<NativeLidarCoverage> lidar;  ///< Required exactly when LiDAR participates in this mode.
    std::chrono::milliseconds retirement_timeout{2000};  ///< Old readers reserve their channels until actual cleanup.
};

/** @brief Platform-owned localization and mapping operations; never synthetic receipts. */
struct NativeExplorationPlatform {
    using MappingServiceFactory = std::function<ExplorationServices(
        const proto::ExplorationGoal&, const ExplorationSessionOptions&)>;  ///< Fresh versioned mapping services per session.
    MappingServiceFactory create_mapping_services;  ///< Returns actual refresh/save/cancel and optional reprojection operations.
    std::function<bool(const std::string&)> prepare_mapping;  ///< Confirms the actual mapping session when requested by the goal.
    perception::ObservationIngress::LocalizationLookup localization;  ///< Real healthy poses and source support at acquisition time.
    perception::ObservationIngress::Clock clock;  ///< Same time domain as native sensor, pose and map publication stamps.
    ExplorationPacketProcessor process_sensor_packet;  ///< Actual owner-side mapper operation; bootstrap inputs carry no session fusion cut.
};

/** @brief Install one native factory and source configuration on TaskServer's idle task.
 * @param task Shared owner returned by TaskServer::exploration().
 * @param node Exact communication node already injected into this task.
 * @param configuration Explicit mode, calibration, execution and route settings.
 * @param platform Real localization/mapper providers with session-owned outstanding requests.
 * @return False for a busy task or a different node, without partial installation.
 * @throws std::invalid_argument Missing providers or inconsistent mode/calibration/settings.
 * @details Creates no graph and starts no Session on the calling thread. The task
 * owner later builds one native route client shared by coverage and pose adaptation,
 * obtains its own guarded navigation dispatcher, and returns an unstarted BT Session.
 * No default TF seed, receiver count, map copy or accepted write becomes a fusion
 * or persistence receipt. Platform callbacks must provide those real operations.
 */
bool ConfigureNativeExploration(const std::shared_ptr<ExplorationTask>& task,
    const std::shared_ptr<autolink::Node>& node, NativeExplorationConfiguration configuration,
    NativeExplorationPlatform platform);

}  // namespace autonomy::task::exploration
