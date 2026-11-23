/* Copyright 2026 The Openbot Authors. */
/** @file native_atlas_localization.hpp
 * @brief Correlate actual Atlas diagnostics and odometry for acquisition-time observation admission.
 */
#pragma once

#include <memory>
#include <optional>

#include <automsgs/msgs/diagnostic_msgs/diagnostic_array.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/perception/observation/observation_ingress.hpp"

namespace autonomy::perception {

/** @brief Fixed native endpoints and actual camera installation, independent from a task generation. */
struct NativeAtlasLocalizationOptions {
    std::string backend_frame;  ///< Actual Atlas world frame, possibly an optical-frame world.
    std::string navigation_frame;  ///< Explicit task/planner world frame after platform-owned alignment.
    std::string camera_frame;  ///< Exact tracked RGB-D optical frame.
    std::string base_frame;  ///< Actual robot frame to which the installation transform refers.
    std::string odometry_topic;  ///< Endpoint declared by the actual diagnostic producer.
    ::automsgs::msgs::geometry_msgs::TransformStamped base_from_camera;  ///< Actual static installation, not a default TF seed.
    std::uint64_t calibration_revision = 0;  ///< Nonzero platform authority for the physical installation/intrinsics.
    std::size_t maximum_samples = 128;  ///< Bound for complete and unmatched integer-stamped samples.
    double evidence_lifetime = .5;  ///< Expiration after acquisition in the source clock domain, seconds.
};

/** @brief Bounded native-message correlation without owning a SLAM instance or inventing a pose.
 * The platform feeds the configured DiagnosticArray and Odometry readers here.
 * No proof is available before a matching healthy diagnostic and real pose exist,
 * and before the platform installs an explicit navigation-from-backend transform.
 * A backend restart or coordinate identity change revokes that alignment. All
 * methods are thread safe; they neither tick a task nor issue movement.
 */
class NativeAtlasLocalization {
public:
    /** @brief Validate fixed frames, calibration and bounded evidence policy. */
    explicit NativeAtlasLocalization(NativeAtlasLocalizationOptions options);
    ~NativeAtlasLocalization();
    NativeAtlasLocalization(const NativeAtlasLocalization&) = delete;
    NativeAtlasLocalization& operator=(const NativeAtlasLocalization&) = delete;
    /** @brief Admit one actual per-acquisition tracker diagnostic, retaining raw backend identities.
     * @return False for malformed metadata, contradictory equal-time status, or an obsolete backend identity.
     */
    bool PushStatus(const ::automsgs::msgs::diagnostic_msgs::DiagnosticArray& status);
    /** @brief Admit an integer-stamped actual estimated camera pose from the configured endpoint. */
    bool PushOdometry(const ::automsgs::msgs::nav_msgs::Odometry& odometry);
    /** @brief Read the newest matching healthy backend pose for explicit platform alignment.
     * @param now Current source-clock time; expired samples are unavailable.
     * @return The original automsgs message, preserving its integer acquisition Header.
     */
    std::optional<::automsgs::msgs::nav_msgs::Odometry> LatestBackendPose(double now) const;
    /** @brief Diagnostic summary of the newest stored sample for one acquisition window. */
    std::string LatestSampleSummary(double now) const;
    /** @brief Install an explicit platform-derived alignment anchored to that newest healthy acquisition.
     * @param alignment Transform with navigation parent, backend child and the exact anchor stamp.
     * @param now Current source-clock time; stale anchors are rejected.
     * @return False without changing state for a wrong frame/stamp or attempted in-epoch realignment.
     * @details The platform must derive this from its real initialization and
     * installation convention. This class never fits GT or seeds an identity pose.
     */
    bool SetWorldAlignment(const ::automsgs::msgs::geometry_msgs::TransformStamped& alignment, double now);
    /** @brief Return actual same-acquisition camera/base evidence for the current backend epoch.
     * No nearest-time matching, interpolation or newest-pose substitution is used.
     * Ambiguous double requests matching two distinct native stamps are rejected.
     */
    std::optional<AcquisitionLocalization> Lookup(SensorKind sensor, const std::string& frame,
                                                double acquisition, double now) const;
    /** @brief Read the current explicit alignment, or null after a backend coordinate change. */
    std::optional<::automsgs::msgs::geometry_msgs::TransformStamped> WorldAlignment() const;
    /** @brief Read the monotonic consumer epoch; raw Atlas map/coordinate ids may legally be zero. */
    std::uint64_t map_epoch() const;
private:
    class Impl;  ///< Mutex-protected bounded correlation and revocable platform alignment.
    std::unique_ptr<Impl> impl_;  ///< Sole mutable authority for this native receiver instance.
};
}  // namespace autonomy::perception
