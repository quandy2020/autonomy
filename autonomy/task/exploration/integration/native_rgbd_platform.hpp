/* Copyright 2026 The Openbot Authors. */
/** @file native_rgbd_platform.hpp
 * @brief Production RGB-D exploration assembly from actual calibration and Atlas messages.
 */
#pragma once

#include "autonomy/perception/observation/native_atlas_localization.hpp"
#include "autonomy/task/exploration/integration/native_observation_mapper.hpp"
#include "autonomy/task/exploration/integration/native_session_factory.hpp"
#include "autonomy/control/native_command_gate.hpp"
#include "autonomy/control/proto/controller_options.pb.h"

namespace autonomy::task::exploration {
/** @brief Actual source endpoints, explicit world convention and existing-server options. */
struct NativeRgbdPlatformOptions {
    NativeExplorationConfiguration exploration;  ///< Actual calibrated native input, coverage, route and execution policy.
    NativeObservationMapperOptions mapping;  ///< Explicit bounded native mapping/persistence configuration.
    perception::NativeAtlasLocalizationOptions localization;  ///< Actual installed camera and native tracking endpoint.
    std::string status_topic;  ///< Actual Atlas DiagnosticArray endpoint for this instance.
    std::string navigation_base_frame;  ///< Isolated TF alias of the calibrated physical base frame.
    std::string odometry_output_topic;  ///< Existing Odometry output after actual base-pose alignment.
    std::string occupancy_output_topic;  ///< Existing OccupancyGrid from actual mapper fusion.
    std::string grid_output_topic;  ///< Existing GridMap from actual native layer storage.
    std::string measured_coverage_topic;  ///< Native target-ledger percentages, distinct from occupancy and whole-room coverage.
    bool host_vista_server = false;  ///< Embed the existing PlannerServer with this mapper as its sole immutable source.
    unsigned vista_edge_confirmation_votes = 0;  ///< Optional FAR style edge hysteresis; 0 keeps the planner default.
    std::optional<control::proto::ControllerOptions> controller;  ///< Actual reviewed existing controller/checker/plugin options; absence leaves execution unconfigured.
    std::string command_topic;  ///< Actual isolated base TwistStamped transport after independent map admission.
    std::string controller_image_topic;  ///< Full original depth relayed with an explicitly calibrated private optical-frame alias.
    std::string controller_camera_frame;  ///< Private optical frame with the same physical installation and intrinsics.
    control::NativeCommandGateOptions command_gate;  ///< Actual body, speed, feedback, watchdog and braking policy.
};

/** @brief Read actual CameraInfo/TF protobuf calibration and explicit YAML settings.
 * @details Supports this RGB-D-only platform. Other sensor modes retain their
 * independent native-source/provider interfaces; no synthetic image path exists.
 */
NativeRgbdPlatformOptions LoadNativeRgbdPlatformOptions(const std::filesystem::path& path);

/** @brief Owns actual Atlas readers, mapper and optional existing planning/control servers.
 * TaskServer remains the task/motion owner. A navigation frame is initialized
 * from an actual healthy camera pose and actual installation transform; GT is
 * not read. No controller is substituted when the requested plugin is absent.
 */
class NativeRgbdPlatform {
public:
    /** @brief Prepare native providers without creating an exploration Session. */
    NativeRgbdPlatform(std::shared_ptr<ExplorationTask> task, std::shared_ptr<autolink::Node> task_node,
                       NativeRgbdPlatformOptions options);
    ~NativeRgbdPlatform();
    /** @brief Install the real factory and source policy, then subscribe actual Atlas endpoints. */
    bool Install();
    /** @brief Close native provider admission and retire only this platform's own readers. */
    void Shutdown();
    /** @brief Verify actual provider-reader retirement within a finite wall deadline. */
    bool WaitForShutdown(std::chrono::milliseconds timeout) const;
private:
    class Impl;  ///< Actual immutable settings, calibrated native localization, mapping and reader lifecycle.
    std::shared_ptr<Impl> impl_;  ///< Pins providers across active callbacks and actual retirement.
};
}  // namespace autonomy::task::exploration
