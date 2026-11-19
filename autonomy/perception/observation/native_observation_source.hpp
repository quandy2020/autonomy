/* Copyright 2026 The Openbot Authors. */
/** @file native_observation_source.hpp
 * @brief Autolink camera/cloud subscriptions feeding correlated native input packets.
 */
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>

#include "autonomy/perception/observation/observation_ingress.hpp"
#include "autonomy/perception/observation/reader_retirement.hpp"

namespace autolink { class Node; }
namespace autonomy::perception {

/** @brief Immutable topic/calibration policy for a native observation source. */
struct NativeObservationSourceOptions {
    SensorMode mode = SensorMode::kRgbdOnly;  ///< Selected input mode.
    SensorDecodingOptions decoding;  ///< Explicit units, beam timing and work bounds.
    std::string depth_topic;  ///< Rectified depth Image channel.
    std::string camera_info_topic;  ///< Matching optical CameraInfo channel.
    std::string lidar_topic;  ///< Raw sensor-frame PointCloud2 channel.
    std::uint32_t pending_queue_size = 32;  ///< Bounded Reader callback backlog, not a no-loss guarantee.
    std::uint64_t depth_calibration_revision = 0;  ///< Authority-assigned revision, including extrinsics.
    bool depth_rectified = false;  ///< Explicit rectification assertion for depth_topic.
    double minimum_depth = .1, maximum_depth = 8;  ///< Valid axial depth range in meters.
    std::optional<CameraModel> expected_depth_camera;  ///< When set, coverage and actual CameraInfo must use these exact intrinsics.
    std::chrono::milliseconds localization_wait_timeout{0};  ///< Explicit wall-time wait for delayed acquisition localization; zero preserves immediate admission.
    std::chrono::milliseconds localization_retry_interval{5};  ///< Minimum wall-time spacing between unresolved-pose lookups.
    std::uint32_t localization_queue_size = 8;  ///< Maximum queued/in-flight messages per sensor, independent from the native Reader backlog.
    std::size_t localization_queue_bytes = 32 * 1024 * 1024;  ///< Per-sensor serialized-payload retention bound, including the in-flight head.
};

/** @brief Validate immutable source settings before installing any native Reader.
 * @param options Topics, calibration, ranges and bounded decoding policy.
 * @throws std::invalid_argument For an unsupported mode or inconsistent configuration.
 */
void ValidateNativeObservationSourceOptions(const NativeObservationSourceOptions& options);

/** @brief Receiver-side counters; they do not prove upstream delivery completeness. */
struct NativeObservationSourceSnapshot {
    std::array<std::uint64_t, 2> received{0, 0};  ///< Local callback ordinals per source.
    std::array<std::uint64_t, 2> accepted{0, 0};  ///< Correlated packets admitted by the sink.
    std::array<std::uint64_t, 2> rejected{0, 0};  ///< Invalid/degraded/overloaded input callbacks.
    std::array<std::array<std::uint64_t, 8>, 2> rejected_by_reason{};  ///< Actual ingress result categories per source, excluding successful admission.
    bool running = false;  ///< All configured readers were created and admission is open.
    bool calibration_invalidated = false;  ///< Different calibration requires an explicit new revision/source.
    bool depth_calibration_ready = false;  ///< Actual matching CameraInfo has been accepted by this source, not inferred from Reader creation.
    std::array<std::size_t, 2> awaiting_localization{0, 0};  ///< Locally retained source inputs; never a fusion or coverage receipt.
    std::array<std::uint64_t, 2> localization_timeouts{0, 0};  ///< Inputs whose acquisition proof remained unavailable through the wall-time deadline.
    std::array<std::uint64_t, 2> localization_overflows{0, 0};  ///< Newly arriving inputs rejected without displacing retained source ordinals.
    std::array<std::uint64_t, 2> localization_canceled{0, 0};  ///< Retained inputs released by source retirement, distinct from a fusion result or admitted packet.
    ReaderRetirementSnapshot retirement;  ///< Requested cleanup, completion and actual success.
};

/** @brief Own native readers without owning SLAM, map or motion.
 * Sink/loss bind to one ExplorationTask generation. Lookup must be provided by
 * the configured localization chain; default TF seeds do not attest health.
 * Optional delayed admission retains the original message stamp and calibration
 * on a bounded worker queue. It never substitutes a latest pose or extends sensor
 * evidence expiry. Lookup and sink must tolerate off-callback worker invocation.
 * Native Header has no acquisition sequence, so receiver ordinals cannot prove
 * that upstream frames or packets were never lost.
 * The caller reserves these Node channels for the source until retirement ends;
 * unrelated code must not concurrently replace their Reader registrations.
 */
class NativeObservationSource {
public:
    /** @brief Notify the task of a rejected local source ordinal even under overload. */
    using LossSink = std::function<void(SensorKind, std::uint64_t)>;
    /** @brief Construct a source with a node and independently attested localization.
     * @throws std::invalid_argument For missing topics/calibration or invalid policy.
     */
    NativeObservationSource(std::shared_ptr<autolink::Node> node,
        NativeObservationSourceOptions options,
        ObservationIngress::LocalizationLookup localization,
        ObservationIngress::Sink sink, ObservationIngress::Clock clock, LossSink loss);
    /** @brief Close admission before releasing readers and weak callback state. */
    ~NativeObservationSource();
    NativeObservationSource(const NativeObservationSource&) = delete;
    NativeObservationSource& operator=(const NativeObservationSource&) = delete;
    /** @brief Create all configured readers; failure leaves this source closed. */
    bool Start();
    /** @brief Revoke admission and request cleanup off native callback threads.
     * This does not wait for Reader::Shutdown or declare physical input drained.
     */
    void Shutdown();
    /** @brief Audit successful Reader shutdown/unregistration within a deadline.
     * @return False on timeout/failure, a native coroutine, or this source's association worker.
     * @pre Shutdown was requested and the caller owns the reserved Node channels.
     */
    bool WaitForShutdown(std::chrono::milliseconds timeout) const;
    /** @brief Retain cleanup accounting even if the final Source reference dies in a callback.
     * The future becomes ready only after actual Reader cleanup and capture release.
     */
    std::shared_future<bool> ShutdownCompletion() const;
    /** @brief Read receiver-side counts and calibration status consistently. */
    NativeObservationSourceSnapshot Snapshot() const;
private:
    class Impl;  ///< Weak callback state, per-source ordering and reader ownership.
    std::shared_ptr<Impl> impl_;  ///< Pinned by source, active callbacks and cleanup; registrations capture weak references.
    std::shared_ptr<ReaderRetirement> retirement_;  ///< Pins native resources until off-callback cleanup ends.
};
}  // namespace autonomy::perception
