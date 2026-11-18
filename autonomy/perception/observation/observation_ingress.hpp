/* Copyright 2026 The Openbot Authors. */
/** @file observation_ingress.hpp
 * @brief Real measurement decoding with acquisition-time localization admission.
 */
#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "autonomy/perception/observation/sensor_message_decoder.hpp"

namespace autonomy::perception {

/** @brief Authoritative source localization at one actual acquisition time.
 * A TF lookup alone is insufficient: the localization owner must attest health,
 * support, alignment and the calibration version used by this measurement.
 */
struct AcquisitionLocalization {
    RigidTransform world_from_sensor;  ///< Sensor pose at the requested acquisition time.
    RigidTransform world_from_base;  ///< Robot pose at the same time and alignment.
    std::string world_frame;  ///< Shared map frame of both transforms.
    std::string sensor_frame;  ///< Exact sensor frame requested from the localization owner.
    std::uint64_t map_epoch = 0;  ///< Nonzero consumer alignment identity.
    std::uint64_t calibration_revision = 0;  ///< Nonzero calibration identity, including extrinsics.
    SensorMask support = 0;  ///< Sensors actually supporting this healthy localization.
    double valid_from = 0, valid_until = 0;  ///< Attested temporal support interval, seconds.
    bool healthy = false;  ///< Explicit localization-owner health, not message arrival.
    /** @brief Verify frame, time, transform and source support together. */
    bool Valid(SensorKind sensor, const std::string& requested_frame,
               double acquisition_time, double now) const;
};

/** @brief Actual base/sensor localization at one beam acquisition time. */
struct AcquisitionPose {
    double stamp = 0;  ///< Beam acquisition time in the shared clock domain.
    AcquisitionLocalization localization;  ///< Independently attested pose at stamp.
};

/** @brief Owned depth input retaining localization, calibration and expiry. */
struct LocalizedDepthView {
    DepthView view;  ///< Axial metric depth and explicit validity mask.
    AcquisitionLocalization localization;  ///< Pose and calibration of this view.
    double valid_until = 0;  ///< Minimum sensor/localization evidence expiration.
    /** @brief Check the view and its provenance before feature consumption. */
    bool Valid(double now) const;
};

/** @brief One owned input correlating real base pose and sensor evidence.
 * Admission succeeds only as a single packet. A packet is not a coverage,
 * fusion, execution or save receipt.
 */
struct ObservationPacket {
    AcquisitionLocalization localization;  ///< Matching localization at observation.stamp.
    SensorObservation observation;  ///< Measured world-frame rays with source/time identity.
    std::optional<DepthView> depth_view;  ///< Native depth view for later frozen DeFM encoding.
    /** @brief LiDAR pose proofs ordered by acquisition time; empty for RGB-D.
     * Every emitted LiDAR ray needs a proof at its exact stamp and matching origin.
     * Duplicate stamps must attest identical sensor/base transforms. Proofs at the
     * observation stamp must also agree with localization. They are real owner
     * attestations, not interpolated guesses made by the input adapter.
     */
    std::vector<AcquisitionPose> acquisition_poses;
    /** @brief Check identity, time, complete beam proofs and ray origins at admission.
     * @param now Actual admission or owner-consumption time in the source clock domain.
     * @return True only when every emitted ray has matching valid localization evidence.
     */
    bool Valid(double now) const;
};

/** @brief Why a source message was admitted or rejected. */
enum class ObservationIngressResult {
    kAccepted, kClosed, kWrongMode, kInvalidMessage, kInvalidLocalization,
    kInvalidEvidence, kSinkRejected, kDependencyFailure
};

/** @brief Pure native input pipeline; sources provide bytes and actual localization.
 * Calls may run concurrently when the supplied lookup/sink/clock are safe.
 * A source must serialize its own sequence assignment and delivery order.
 * Close revokes future admission without waiting for a producer callback.
 */
class ObservationIngress {
public:
    /** @brief Lookup actual acquisition-time sensor/base poses and health. */
    using LocalizationLookup = std::function<std::optional<AcquisitionLocalization>(
        SensorKind, const std::string&, double)>;
    /** @brief Bounded atomic packet admission; must not tick a Session or issue motion. */
    using Sink = std::function<bool(ObservationPacket, double)>;
    /** @brief Acquisition/owner-compatible current time, in seconds. */
    using Clock = std::function<double()>;

    /** @brief Construct a fixed mode with real localization and input destination.
     * @throws std::invalid_argument For unknown mode, malformed policy or missing dependencies.
     */
    ObservationIngress(SensorMode mode, SensorDecodingOptions options,
        LocalizationLookup localization, Sink sink, Clock clock);
    /** @brief Revoke new input; in-flight decoding rechecks before destination admission. */
    void Close() noexcept;
    /** @brief Decode and admit one depth message with a nonzero source ordinal. */
    ObservationIngressResult PushDepth(const DepthImageBytes& image,
        const DepthCalibration& calibration, std::uint64_t sequence) const;
    /** @brief Decode and admit one cloud with actual per-beam poses. */
    ObservationIngressResult PushCloud(const PointCloudBytes& cloud,
        std::uint64_t sequence) const;

private:
    SensorMode mode_;  ///< Fixed configured source policy.
    SensorDecodingOptions options_;  ///< Fixed finite decoding and work bounds.
    LocalizationLookup localization_;  ///< Trusted, timestamped localization service.
    Sink sink_;  ///< Single packet destination; acceptance is only queue admission.
    Clock clock_;  ///< Shared acquisition/owner clock domain.
    std::atomic<bool> closed_{false};  ///< Revocation flag checked again before emission.
    /** @brief Revalidate time and identity immediately before bounded admission. */
    ObservationIngressResult Emit(ObservationPacket packet) const;
};

}  // namespace autonomy::perception
