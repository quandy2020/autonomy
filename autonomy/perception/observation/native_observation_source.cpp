/* Copyright 2026 The Openbot Authors. */
/** @file native_observation_source.cpp
 * @brief Native readers, source ordering and explicit calibration invalidation.
 */
#include "autonomy/perception/observation/native_observation_source.hpp"

#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>

#include "autolink/autolink.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autonomy/perception/observation/native_sensor_messages.hpp"

namespace autonomy::perception {
class NativeObservationSource::Impl {
public:
    /** @brief Pin immutable dependencies without starting callbacks. */
    Impl(std::shared_ptr<autolink::Node> node, NativeObservationSourceOptions options,
        ObservationIngress::LocalizationLookup lookup, ObservationIngress::Sink sink,
        ObservationIngress::Clock clock, LossSink loss)
        : node_(std::move(node)), options_(std::move(options)),
          ingress_(options_.mode, options_.decoding, std::move(lookup), std::move(sink), std::move(clock)),
          loss_(std::move(loss)) {
        if (!node_ || !loss_ || !options_.pending_queue_size ||
            options_.pending_queue_size > 4096 || ((Mask(options_.mode) & Mask(SensorKind::kRgbd)) &&
            (options_.depth_topic.empty() || options_.camera_info_topic.empty() ||
             options_.depth_topic == options_.camera_info_topic ||
             !options_.depth_calibration_revision || !options_.depth_rectified ||
             !std::isfinite(options_.minimum_depth) || options_.minimum_depth <= 0 ||
             !std::isfinite(options_.maximum_depth) || options_.maximum_depth <= options_.minimum_depth)) ||
            ((Mask(options_.mode) & Mask(SensorKind::kLidar)) && options_.lidar_topic.empty()) ||
            (options_.mode == SensorMode::kLidarRgbd &&
             (options_.lidar_topic == options_.depth_topic || options_.lidar_topic == options_.camera_info_topic)))
            throw std::invalid_argument("native observation source requires real topics and depth calibration");
    }
    /** @brief Prepare an actual ReaderConfig with an explicit callback backlog. */
    autolink::ReaderConfig ReaderOptions(const std::string& topic) const {
        autolink::ReaderConfig config;
        config.channel_name = topic;
        config.pending_queue_size = options_.pending_queue_size;
        return config;
    }
    /** @brief Revoke immediately without waiting for native scheduler locks. */
    void Close() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        closed_ = true; running_.store(false); ingress_.Close();
    }
    /** @brief Permanent dependency failure also schedules Reader retirement off-callback. */
    void Fail() {
        Close();
        if (const auto retirement = retirement_.lock()) retirement->Request();
    }
    /** @brief Stop a pinned Reader before removing its own Node registration.
     * @pre Reserved channels cannot be concurrently replaced by external code.
     * GetReader prevents deleting a registration already known to be different;
     * the existing Node API has no atomic compare-and-delete operation.
     */
    template <typename Message>
    bool StopReader(const std::string& topic, const std::shared_ptr<autolink::Reader<Message>>& reader) {
        if (!reader) return true;
        reader->Shutdown();
        if (node_->GetReader<Message>(topic) == reader) return node_->DeleteReader(topic);
        return true;
    }
    /** @brief Drain and release native readers on the dedicated cleanup thread.
     * Scheduler removal may wait for callbacks; no source lifecycle lock is held.
     */
    bool RetireReaders() {
        bool success = true;
        try { if (!StopReader(options_.depth_topic, depth_)) success = false; } catch (...) { success = false; }
        try { if (!StopReader(options_.camera_info_topic, calibration_reader_)) success = false; } catch (...) { success = false; }
        try { if (!StopReader(options_.lidar_topic, cloud_)) success = false; } catch (...) { success = false; }
        if (success) {
            std::lock_guard<std::mutex> lock(lifecycle_mutex_);
            depth_.reset(); calibration_reader_.reset(); cloud_.reset(); node_.reset();
        }
        return success;
    }
    /** @brief Latch the first calibration; changed intrinsics need a new revision. */
    void Calibration(const std::shared_ptr<::automsgs::msgs::sensor_msgs::CameraInfo>& message) {
        if (!message || !running_.load()) return;
        std::optional<DepthCalibration> candidate;
        try {
            candidate = NativeDepthCalibration(*message, options_.depth_calibration_revision,
                options_.depth_rectified, options_.minimum_depth, options_.maximum_depth);
        } catch (...) {
            Fail();
            return;
        }
        std::lock_guard<std::mutex> lock(calibration_mutex_);
        if (!candidate || calibration_invalidated_) return;
        if (calibration_) {
            const auto& a = calibration_->camera; const auto& b = candidate->camera;
            if (calibration_->frame_id != candidate->frame_id || a.width != b.width || a.height != b.height ||
                a.fx != b.fx || a.fy != b.fy || a.cx != b.cx || a.cy != b.cy) {
                calibration_invalidated_ = true; calibration_.reset(); return;
            }
        }
        calibration_ = std::move(candidate);
    }
    /** @brief Reserve a receiver ordinal before decoding, including rejected frames. */
    std::uint64_t Sequence(unsigned index) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (stats_.received[index] == std::numeric_limits<std::uint64_t>::max()) return 0;
        return ++stats_.received[index];
    }
    /** @brief Record actual sink admission without turning it into coverage. */
    void Result(unsigned index, ObservationIngressResult result) {
        std::uint64_t sequence;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            auto& counter = result == ObservationIngressResult::kAccepted ? stats_.accepted[index] : stats_.rejected[index];
            if (counter != std::numeric_limits<std::uint64_t>::max()) ++counter;
            sequence = stats_.received[index];
        }
        if (result != ObservationIngressResult::kAccepted) {
            try { loss_(index == 0 ? SensorKind::kLidar : SensorKind::kRgbd, sequence); }
            catch (...) { Fail(); }
        }
    }
    /** @brief Serialize depth assignment/decoding/delivery for this source only. */
    void Depth(const std::shared_ptr<::automsgs::msgs::sensor_msgs::Image>& message) {
        std::lock_guard<std::mutex> lock(source_mutex_[1]);
        if (!running_.load()) return;
        const auto sequence = Sequence(1);
        if (!sequence) { Fail(); return; }
        auto result = ObservationIngressResult::kDependencyFailure;
        try {
            std::optional<DepthCalibration> calibration;
            { std::lock_guard<std::mutex> calibration_lock(calibration_mutex_); calibration = calibration_; }
            auto image = message ? NativeDepthImage(*message) : std::optional<DepthImageBytes>{};
            result = image && calibration ? ingress_.PushDepth(*image, *calibration, sequence) :
                ObservationIngressResult::kInvalidMessage;
        } catch (...) {}
        Result(1, result);
    }
    /** @brief Serialize LiDAR callbacks while independently allowing depth input. */
    void Cloud(const std::shared_ptr<::automsgs::msgs::sensor_msgs::PointCloud2>& message) {
        std::lock_guard<std::mutex> lock(source_mutex_[0]);
        if (!running_.load()) return;
        const auto sequence = Sequence(0);
        if (!sequence) { Fail(); return; }
        auto result = ObservationIngressResult::kDependencyFailure;
        try {
            auto cloud = message ? NativePointCloud(*message) : std::optional<PointCloudBytes>{};
            result = cloud ? ingress_.PushCloud(*cloud, sequence) : ObservationIngressResult::kInvalidMessage;
        } catch (...) {}
        Result(0, result);
    }
    std::shared_ptr<autolink::Node> node_;  ///< Actual native node; no ROS bridge.
    NativeObservationSourceOptions options_;  ///< Immutable topics and calibration/timing policy.
    ObservationIngress ingress_;  ///< Checked decoder/localization/atomic destination pipeline.
    LossSink loss_;  ///< Required task loss channel, independent from ordinary queue capacity.
    std::weak_ptr<ReaderRetirement> retirement_;  ///< Fatal callbacks request cleanup without a worker/Impl ownership cycle.
    std::atomic<bool> running_{false};  ///< Admission opens only after all readers exist.
    bool closed_ = false;  ///< Lifecycle-lock-protected permanent close/failure flag.
    mutable std::mutex lifecycle_mutex_;  ///< Serializes Start/Shutdown reader transitions.
    std::array<std::mutex, 2> source_mutex_;  ///< Independent ordered source callbacks.
    mutable std::mutex calibration_mutex_;  ///< Guards the pinned depth calibration.
    std::optional<DepthCalibration> calibration_;  ///< First valid calibration for this revision.
    bool calibration_invalidated_ = false;  ///< Changed intrinsics stay invalid until explicit reconfiguration.
    mutable std::mutex stats_mutex_;  ///< Guards receiver-side counters.
    NativeObservationSourceSnapshot stats_;  ///< Counters independent from mapper/coverage state.
    std::shared_ptr<autolink::Reader<::automsgs::msgs::sensor_msgs::Image>> depth_;  ///< Depth reader.
    std::shared_ptr<autolink::Reader<::automsgs::msgs::sensor_msgs::CameraInfo>> calibration_reader_;  ///< Optical calibration reader.
    std::shared_ptr<autolink::Reader<::automsgs::msgs::sensor_msgs::PointCloud2>> cloud_;  ///< Raw timed cloud reader.
};

NativeObservationSource::NativeObservationSource(std::shared_ptr<autolink::Node> node,
    NativeObservationSourceOptions options, ObservationIngress::LocalizationLookup lookup,
    ObservationIngress::Sink sink, ObservationIngress::Clock clock, LossSink loss)
    : impl_(std::make_shared<Impl>(std::move(node), std::move(options), std::move(lookup),
                                  std::move(sink), std::move(clock), std::move(loss))),
      retirement_(std::make_shared<ReaderRetirement>([state = impl_] { return state->RetireReaders(); })) {
    impl_->retirement_ = retirement_;
}
NativeObservationSource::~NativeObservationSource() { Shutdown(); }

bool NativeObservationSource::Start() {
    std::unique_lock<std::mutex> lock(impl_->lifecycle_mutex_);
    if (impl_->closed_) return false;
    if (impl_->running_.load()) return true;
    const std::weak_ptr<Impl> weak = impl_;
    try {
        if (Mask(impl_->options_.mode) & Mask(SensorKind::kRgbd)) {
            impl_->calibration_reader_ = impl_->node_->CreateReader<::automsgs::msgs::sensor_msgs::CameraInfo>(
                impl_->ReaderOptions(impl_->options_.camera_info_topic), [weak](const auto& message) {
                    if (auto self = weak.lock()) self->Calibration(message);
                });
            impl_->depth_ = impl_->node_->CreateReader<::automsgs::msgs::sensor_msgs::Image>(
                impl_->ReaderOptions(impl_->options_.depth_topic), [weak](const auto& message) {
                    if (auto self = weak.lock()) self->Depth(message);
                });
            if (!impl_->calibration_reader_ || !impl_->depth_) throw std::runtime_error("depth reader creation failed");
        }
        if (Mask(impl_->options_.mode) & Mask(SensorKind::kLidar)) {
            impl_->cloud_ = impl_->node_->CreateReader<::automsgs::msgs::sensor_msgs::PointCloud2>(
                impl_->ReaderOptions(impl_->options_.lidar_topic), [weak](const auto& message) {
                    if (auto self = weak.lock()) self->Cloud(message);
                });
            if (!impl_->cloud_) throw std::runtime_error("cloud reader creation failed");
        }
        impl_->running_.store(true);
        return true;
    } catch (...) {
        impl_->closed_ = true; impl_->ingress_.Close();
        impl_->running_.store(false);
        lock.unlock();
        retirement_->Request();
        return false;
    }
}

void NativeObservationSource::Shutdown() {
    impl_->Close();
    retirement_->Request();
}
bool NativeObservationSource::WaitForShutdown(std::chrono::milliseconds timeout) const {
    if (autolink::croutine::CRoutine::GetCurrentRoutine()) return false;
    return retirement_->WaitFor(timeout);
}
std::shared_future<bool> NativeObservationSource::ShutdownCompletion() const {
    return retirement_->Completion();
}

NativeObservationSourceSnapshot NativeObservationSource::Snapshot() const {
    NativeObservationSourceSnapshot result;
    { std::lock_guard<std::mutex> lock(impl_->stats_mutex_); result = impl_->stats_; }
    { std::lock_guard<std::mutex> lock(impl_->calibration_mutex_); result.calibration_invalidated = impl_->calibration_invalidated_; }
    result.running = impl_->running_.load();
    result.retirement = retirement_->Snapshot();
    return result;
}
}  // namespace autonomy::perception
