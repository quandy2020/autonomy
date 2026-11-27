/* Copyright 2026 The Openbot Authors. */
/** @file native_observation_source.cpp
 * @brief Native readers, source ordering and explicit calibration invalidation.
 */
#include "autonomy/perception/observation/native_observation_source.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include "autonomy/common/logging.hpp"
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autonomy/perception/observation/native_sensor_messages.hpp"

namespace autonomy::perception {
namespace {
/** @brief Detect self-waits without racing std::thread handle publication. */
thread_local const void* association_owner = nullptr;
}  // namespace

void ValidateNativeObservationSourceOptions(const NativeObservationSourceOptions& options) {
    ValidateSensorDecodingOptions(options.decoding);
    if (options.mode != SensorMode::kRgbdOnly && options.mode != SensorMode::kLidarOnly &&
        options.mode != SensorMode::kLidarRgbd)
        throw std::invalid_argument("unsupported native observation source mode");
    const bool depth = (Mask(options.mode) & Mask(SensorKind::kRgbd)) != 0;
    const bool lidar = (Mask(options.mode) & Mask(SensorKind::kLidar)) != 0;
    if (!options.pending_queue_size || options.pending_queue_size > 4096 ||
        options.localization_wait_timeout < std::chrono::milliseconds::zero() ||
        options.localization_wait_timeout > std::chrono::seconds(5) ||
        options.localization_retry_interval <= std::chrono::milliseconds::zero() ||
        options.localization_retry_interval > std::chrono::seconds(1) ||
        !options.localization_queue_size || options.localization_queue_size > 4096 ||
        !options.localization_queue_bytes || options.localization_queue_bytes > 256 * 1024 * 1024 ||
        (depth && (options.depth_topic.empty() || options.camera_info_topic.empty() ||
         options.depth_topic == options.camera_info_topic || !options.depth_calibration_revision ||
         !options.depth_rectified || !std::isfinite(options.minimum_depth) || options.minimum_depth <= 0 ||
         !std::isfinite(options.maximum_depth) || options.maximum_depth <= options.minimum_depth)) ||
        (lidar && options.lidar_topic.empty()) ||
        (depth && lidar && (options.lidar_topic == options.depth_topic ||
                           options.lidar_topic == options.camera_info_topic)))
        throw std::invalid_argument("native observation source requires real topics and depth calibration");
    if (options.expected_depth_camera && (!depth || !options.expected_depth_camera->Valid() ||
        options.expected_depth_camera->min_depth != options.minimum_depth ||
        options.expected_depth_camera->max_depth != options.maximum_depth))
        throw std::invalid_argument("expected camera must match the configured native depth range");
}

class NativeObservationSource::Impl {
public:
    /** @brief Pin immutable dependencies without starting callbacks. */
    Impl(std::shared_ptr<autolink::Node> node, NativeObservationSourceOptions options,
        ObservationIngress::LocalizationLookup lookup, ObservationIngress::Sink sink,
        ObservationIngress::Clock clock, LossSink loss)
        : node_(std::move(node)), options_(std::move(options)),
          lookup_(std::move(lookup)), clock_(std::move(clock)),
          ingress_(options_.mode, options_.decoding, lookup_, std::move(sink), clock_),
          loss_(std::move(loss)) {
        ValidateNativeObservationSourceOptions(options_);
        if (!node_ || !loss_) throw std::invalid_argument("native observation source requires a Node and loss sink");
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
        closed_ = true;
        {
            std::lock_guard<std::mutex> pending_lock(pending_mutex_);
            running_.store(false); ingress_.Close();
        }
        pending_ready_.notify_all();
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
        // Joining occurs only on ReaderRetirement's cleanup thread. A sink may
        // request Shutdown, but cannot wait for the worker invoking that sink.
        for (auto& worker : association_workers_) {
            try { if (worker.joinable()) worker.join(); } catch (...) { success = false; }
        }
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            for (unsigned index = 0; index < 2; ++index)
                stats_.localization_canceled[index] += pending_[index].size();
            for (auto& queue : pending_) queue.clear();
            pending_bytes_ = {0, 0};
        }
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
        if (options_.expected_depth_camera) {
            const auto& a = *options_.expected_depth_camera; const auto& b = candidate->camera;
            if (a.width != b.width || a.height != b.height || a.fx != b.fx || a.fy != b.fy ||
                a.cx != b.cx || a.cy != b.cy) {
                calibration_invalidated_ = true; calibration_.reset(); return;
            }
        }
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
    void Result(unsigned index, std::uint64_t sequence, ObservationIngressResult result) {
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            auto& counter = result == ObservationIngressResult::kAccepted ? stats_.accepted[index] : stats_.rejected[index];
            if (counter != std::numeric_limits<std::uint64_t>::max()) ++counter;
            const auto reason = static_cast<unsigned>(result);
            if (result != ObservationIngressResult::kAccepted && reason < stats_.rejected_by_reason[index].size() &&
                stats_.rejected_by_reason[index][reason] != std::numeric_limits<std::uint64_t>::max())
                ++stats_.rejected_by_reason[index][reason];
        }
        if (result != ObservationIngressResult::kAccepted &&
            result != ObservationIngressResult::kInvalidEvidence) {
            try { loss_(index == 0 ? SensorKind::kLidar : SensorKind::kRgbd, sequence); }
            catch (...) { Fail(); }
        }
        // Bounded ingress summary: explains a silent observation stall without
        // turning every frame into a log line.
        std::uint64_t received = 0, accepted = 0, rejected = 0, overflows = 0;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            received = stats_.received[index];
            accepted = stats_.accepted[index];
            rejected = stats_.rejected[index];
            overflows = stats_.localization_overflows[index];
        }
        if ((received % 100) == 0) {
            std::size_t pending = 0;
            {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                pending = pending_[index].size();
            }
            AINFO_EVERY(2) << "NATIVE_INGRESS_SUMMARY sensor=" << index
                           << " received=" << received
                           << " accepted=" << accepted
                           << " rejected=" << rejected
                           << " overflow=" << overflows
                           << " pending=" << pending
                           << " last_result=" << static_cast<int>(result);
        }
    }
    /** @brief Original wire message and calibration retained through delayed lookup. */
    struct PendingInput {
        std::shared_ptr<const ::automsgs::msgs::sensor_msgs::Image> depth;  ///< Owned original depth bytes, with unchanged stamp.
        std::shared_ptr<const ::automsgs::msgs::sensor_msgs::PointCloud2> cloud;  ///< Owned original timed beam records.
        std::optional<DepthCalibration> calibration;  ///< Calibration pinned when the depth input was received.
        std::uint64_t sequence = 0;  ///< Receiver ordinal reserved before queue admission.
        std::size_t bytes = 0;  ///< Serialized retention charge for this input.
        std::chrono::steady_clock::time_point deadline;  ///< Finite wall-time association deadline, never an evidence expiry extension.
    };
    /** @brief Reject overflow without replacing any earlier retained source input. */
    void Enqueue(unsigned index, PendingInput input) {
        bool overflow = false;
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (!running_.load()) return;
            overflow = pending_[index].size() >= options_.localization_queue_size ||
                input.bytes > options_.localization_queue_bytes - pending_bytes_[index];
            if (!overflow) {
                input.deadline = std::chrono::steady_clock::now() + options_.localization_wait_timeout;
                pending_[index].push_back(std::move(input));
                pending_bytes_[index] += pending_[index].back().bytes;
            }
        }
        if (overflow) {
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                if (stats_.localization_overflows[index] != std::numeric_limits<std::uint64_t>::max())
                    ++stats_.localization_overflows[index];
            }
            Result(index, input.sequence, ObservationIngressResult::kSinkRejected);
        } else pending_ready_.notify_all();
    }
    /** @brief Query only the original acquisition time before repeated depth decoding. */
    ObservationIngressResult TryPending(const PendingInput& input) {
        try {
            if (input.depth) {
                std::optional<DepthCalibration> calibration = input.calibration;
                {
                    std::lock_guard<std::mutex> lock(calibration_mutex_);
                    if (calibration_invalidated_)
                        return ObservationIngressResult::kInvalidMessage;
                    if (!calibration) calibration = calibration_;
                }
                auto image = NativeDepthImage(*input.depth);
                if (!image) return ObservationIngressResult::kInvalidMessage;
                // CameraInfo and Image callbacks can be scheduled in either
                // order. Wait for the actual calibration of this fixed revision
                // instead of dropping an otherwise valid first acquisition.
                if (!calibration) return ObservationIngressResult::kInvalidLocalization;
                auto pose = lookup_(SensorKind::kRgbd, image->frame_id, image->stamp);
                if (!pose || !pose->Valid(SensorKind::kRgbd, image->frame_id, image->stamp, clock_()))
                    return ObservationIngressResult::kInvalidLocalization;
                if (std::chrono::steady_clock::now() >= input.deadline)
                    return ObservationIngressResult::kInvalidLocalization;
                return ingress_.PushDepth(*image, *calibration, input.sequence);
            }
            auto cloud = input.cloud ? NativePointCloud(*input.cloud) : std::optional<PointCloudBytes>{};
            return cloud ? ingress_.PushCloud(*cloud, input.sequence) : ObservationIngressResult::kInvalidMessage;
        } catch (...) { return ObservationIngressResult::kDependencyFailure; }
    }
    /** @brief Preserve FIFO source ordinals while waiting off native callback threads. */
    void Associate(unsigned index) noexcept {
        association_owner = this;
        try {
            for (;;) {
                std::unique_lock<std::mutex> lock(pending_mutex_);
                pending_ready_.wait(lock, [&] { return !running_.load() || !pending_[index].empty(); });
                if (!running_.load()) break;
                const auto input = pending_[index].front();
                lock.unlock();
                const bool expired = std::chrono::steady_clock::now() >= input.deadline;
                const auto result = expired ? ObservationIngressResult::kInvalidLocalization : TryPending(input);
                lock.lock();
                if (!running_.load() && result != ObservationIngressResult::kAccepted) break;
                if ((result == ObservationIngressResult::kInvalidLocalization ||
                     result == ObservationIngressResult::kInvalidEvidence) &&
                    std::chrono::steady_clock::now() < input.deadline) {
                    // Pose publication and packet validity are independent
                    // callbacks. A packet observed in the few-millisecond window
                    // before its matching proof arrives is retryable; dropping it
                    // immediately creates a permanent source gap and forces a
                    // full scope recovery for a transient handoff.
                    pending_ready_.wait_until(lock, std::min(input.deadline,
                        std::chrono::steady_clock::now() + options_.localization_retry_interval),
                        [&] { return !running_.load(); });
                    continue;
                }
                pending_bytes_[index] -= input.bytes;
                pending_[index].pop_front();
                lock.unlock();
                if (result == ObservationIngressResult::kInvalidLocalization) {
                    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                    if (stats_.localization_timeouts[index] != std::numeric_limits<std::uint64_t>::max())
                        ++stats_.localization_timeouts[index];
                }
                Result(index, input.sequence, result);
            }
        } catch (...) { Fail(); }
        association_owner = nullptr;
    }
    /** @brief Serialize depth assignment/decoding/delivery for this source only. */
    void Depth(const std::shared_ptr<::automsgs::msgs::sensor_msgs::Image>& message) {
        std::lock_guard<std::mutex> lock(source_mutex_[1]);
        if (!running_.load()) return;
        const auto sequence = Sequence(1);
        if (!sequence) { Fail(); return; }
        if (options_.localization_wait_timeout > std::chrono::milliseconds::zero()) {
            try {
                PendingInput input;
                input.depth = message; input.sequence = sequence;
                {
                    std::lock_guard<std::mutex> calibration_lock(calibration_mutex_);
                    input.calibration = calibration_;
                }
                if (!message) {
                    Result(1, sequence, ObservationIngressResult::kInvalidMessage); return;
                }
                input.bytes = message->ByteSizeLong();
                Enqueue(1, std::move(input)); return;
            } catch (...) { Result(1, sequence, ObservationIngressResult::kDependencyFailure); return; }
        }
        auto result = ObservationIngressResult::kDependencyFailure;
        try {
            std::optional<DepthCalibration> calibration;
            { std::lock_guard<std::mutex> calibration_lock(calibration_mutex_); calibration = calibration_; }
            auto image = message ? NativeDepthImage(*message) : std::optional<DepthImageBytes>{};
            result = image && calibration ? ingress_.PushDepth(*image, *calibration, sequence) :
                ObservationIngressResult::kInvalidMessage;
        } catch (...) {}
        Result(1, sequence, result);
    }
    /** @brief Serialize LiDAR callbacks while independently allowing depth input. */
    void Cloud(const std::shared_ptr<::automsgs::msgs::sensor_msgs::PointCloud2>& message) {
        std::lock_guard<std::mutex> lock(source_mutex_[0]);
        if (!running_.load()) return;
        const auto sequence = Sequence(0);
        if (!sequence) { Fail(); return; }
        if (options_.localization_wait_timeout > std::chrono::milliseconds::zero()) {
            try {
                if (!message) { Result(0, sequence, ObservationIngressResult::kInvalidMessage); return; }
                PendingInput input;
                input.cloud = message; input.sequence = sequence; input.bytes = message->ByteSizeLong();
                Enqueue(0, std::move(input)); return;
            } catch (...) { Result(0, sequence, ObservationIngressResult::kDependencyFailure); return; }
        }
        auto result = ObservationIngressResult::kDependencyFailure;
        try {
            auto cloud = message ? NativePointCloud(*message) : std::optional<PointCloudBytes>{};
            result = cloud ? ingress_.PushCloud(*cloud, sequence) : ObservationIngressResult::kInvalidMessage;
        } catch (...) {}
        Result(0, sequence, result);
    }
    std::shared_ptr<autolink::Node> node_;  ///< Actual native node; no ROS bridge.
    NativeObservationSourceOptions options_;  ///< Immutable topics and calibration/timing policy.
    ObservationIngress::LocalizationLookup lookup_;  ///< Timestamped authority; delay never changes the requested stamp.
    ObservationIngress::Clock clock_;  ///< Shared source/owner clock for unchanged evidence-expiry checks.
    ObservationIngress ingress_;  ///< Checked decoder/localization/atomic destination pipeline.
    mutable std::mutex pending_mutex_;  ///< Protects both bounded queues and association wake predicates.
    std::condition_variable pending_ready_;  ///< Wakes independent sensor workers for input, deadlines or shutdown.
    std::array<std::deque<PendingInput>, 2> pending_;  ///< FIFO inputs, including each in-flight head.
    std::array<std::size_t, 2> pending_bytes_{0, 0};  ///< Retention charge guarded by pending_mutex_.
    std::array<std::thread, 2> association_workers_;  ///< Created only for configured sensors when deferred admission is enabled.
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
        if (impl_->options_.localization_wait_timeout > std::chrono::milliseconds::zero()) {
            for (unsigned index = 0; index < 2; ++index) {
                const auto sensor = index == 0 ? SensorKind::kLidar : SensorKind::kRgbd;
                if (Mask(impl_->options_.mode) & Mask(sensor))
                    impl_->association_workers_[index] = std::thread([state = impl_, index] { state->Associate(index); });
            }
        }
        return true;
    } catch (...) {
        impl_->closed_ = true; impl_->ingress_.Close();
        { std::lock_guard<std::mutex> pending_lock(impl_->pending_mutex_); impl_->running_.store(false); }
        impl_->pending_ready_.notify_all();
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
    if (autolink::croutine::CRoutine::GetCurrentRoutine() || association_owner == impl_.get()) return false;
    return retirement_->WaitFor(timeout);
}
std::shared_future<bool> NativeObservationSource::ShutdownCompletion() const {
    return retirement_->Completion();
}

NativeObservationSourceSnapshot NativeObservationSource::Snapshot() const {
    NativeObservationSourceSnapshot result;
    { std::lock_guard<std::mutex> lock(impl_->stats_mutex_); result = impl_->stats_; }
    {
        std::lock_guard<std::mutex> lock(impl_->pending_mutex_);
        for (unsigned index = 0; index < 2; ++index) result.awaiting_localization[index] = impl_->pending_[index].size();
    }
    {
        std::lock_guard<std::mutex> lock(impl_->calibration_mutex_);
        result.calibration_invalidated = impl_->calibration_invalidated_;
        result.depth_calibration_ready = impl_->calibration_.has_value() && !impl_->calibration_invalidated_;
    }
    result.running = impl_->running_.load();
    result.retirement = retirement_->Snapshot();
    return result;
}
}  // namespace autonomy::perception
