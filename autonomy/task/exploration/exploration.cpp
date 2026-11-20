/* Copyright 2026 The Openbot Authors. */
/** @file exploration.cpp @brief Native exploration Session/BT ownership. */
#include "autonomy/task/exploration/exploration.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/exploration/plugins/exploration_tree.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace task {
namespace {
namespace e = exploration;
/// Unique task-instance scope prevents request reuse after reconstruction.
std::atomic<std::uint64_t> next_instance{0};
/// Identifies native callback reentry without retaining a task pointer.
thread_local std::uint64_t callback_instance = 0;

/** @brief A native dependency may request close but must never self-join. */
class NativeCallbackScope {
public:
    /// Bind the callback thread to its immutable task-instance identity.
    explicit NativeCallbackScope(std::uint64_t instance)
        : previous_(callback_instance) { callback_instance = instance; }
    /// Restore nested callback context on every exit, including exceptions.
    ~NativeCallbackScope() { callback_instance = previous_; }
private:
    /// Identity of an enclosing callback, if any.
    const std::uint64_t previous_;
};

/** @brief Process nonce prevents request identities repeating after a restart. */
const std::string& ProcessIdentity() {
    static const std::string identity = [] {
        std::random_device random;
        std::ostringstream stream;
        stream << std::hex;
        for (int i = 0; i < 8; ++i) stream << random() << '-';
        return stream.str();
    }();
    return identity;
}

/** @brief Build immutable callbacks without pinning a task on its owner thread. */
ExplorationSensorInputBinding SensorBinding(const std::weak_ptr<ExplorationTask>& weak,
    ExplorationSensorInputIdentity identity, ExplorationPacketProcessor processor = {},
    std::uint64_t input_revision = 0) {
    ExplorationSensorInputBinding binding;
    binding.identity = identity;
    binding.sink = [weak, identity, processor = std::move(processor), input_revision](perception::ObservationPacket packet, double now) {
        const auto destination = weak.lock();
        if (!destination) return false;
        if (input_revision) return destination->PushNativeSensorPacket(identity, std::move(packet), now, processor, input_revision);
        return destination->PushSensorPacket(identity, std::move(packet), now);
    };
    binding.loss = [weak, identity](perception::SensorKind sensor, std::uint64_t sequence) {
        if (const auto destination = weak.lock()) destination->ReportSensorLoss(identity, sensor, sequence);
    };
    return binding;
}

/** @brief Return a validated rectangular task boundary, without centroiding. */
std::optional<e::Bounds2d> Rectangle(
    const ::automsgs::msgs::geometry_msgs::Polygon& area) {
    auto count = area.points_size();
    if (count == 5 && area.points(0).x() == area.points(4).x() &&
        area.points(0).y() == area.points(4).y()) --count;
    if (count != 4) return {};
    e::Bounds2d bounds{std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    double twice_area = 0;
    for (int i = 0; i < count; ++i) {
        const auto& p = area.points(i);
        const auto& q = area.points((i + 1) % count);
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) ||
            (p.x() != q.x() && p.y() != q.y()) ||
            (p.x() == q.x() && p.y() == q.y())) return {};
        bounds.min_x = std::min<double>(bounds.min_x, p.x());
        bounds.min_y = std::min<double>(bounds.min_y, p.y());
        bounds.max_x = std::max<double>(bounds.max_x, p.x());
        bounds.max_y = std::max<double>(bounds.max_y, p.y());
        twice_area += p.x() * q.y() - q.x() * p.y();
    }
    const auto box_area = (bounds.max_x - bounds.min_x) *
                          (bounds.max_y - bounds.min_y);
    if (box_area <= 0 || std::abs(std::abs(twice_area) / 2 - box_area) >
                         1e-6 * std::max(1., box_area)) return {};
    return bounds;
}

}  // namespace

/** @brief Owns inbox synchronization and all mutable Session/BT runtime. */
class ExplorationTask::Impl {
public:
    /** @brief Bind the implementation to its native task lifecycle. */
    explicit Impl(ExplorationTask& owner) : owner_(owner), instance_(++next_instance) {}

    /** @brief Pose event preserves real acquisition time and localization mode. */
    struct PoseInput {
        /// Pose in the current map alignment.
        e::Pose2d pose;
        /// Acquisition time in the configured clock domain.
        double stamp;
        /// Map alignment epoch, not a message publication sequence.
        std::uint64_t epoch;
        /// Sources supporting this localization result.
        perception::SensorMask support;
    };
    /** @brief Observation admission retains acquisition and processing clocks. */
    struct ObservationInput {
        /// Owned real sensor observation.
        perception::SensorObservation observation;
        /// Admission time in the shared clock domain.
        double now;
    };
    /** @brief Source packet remains indivisible across task admission and delivery. */
    struct PacketInput {
        perception::ObservationPacket packet;  ///< Actual pose, rays and optional depth.
        double now;  ///< Original admission time in the configured clock domain.
        ExplorationPacketProcessor processor;  ///< Actual operation captured when this native source was created.
        std::uint64_t input_revision = 0;  ///< Native configuration revision; zero means ordinary injected input.
    };
    /** @brief Two fixed source slots preserve loss even when the data inbox is full. */
    struct SourceLoss {
        std::uint64_t generation;  ///< Task identity at rejection.
        perception::SensorKind sensor;  ///< Rejected configured source.
        std::uint64_t first, last;  ///< Conservative receiver ordinal interval.
    };
    /// Data events are processed in bounded batches on the owner.
    using Data = std::variant<e::MapReadView, PoseInput, ObservationInput, e::CoverageUnit, PacketInput>;
    /** @brief An input belongs to the task generation that admitted it. */
    struct Input {
        /// Zero permits pre-start map/pose seeding, never coverage completion.
        std::uint64_t generation;
        /// Owned immutable event payload.
        Data data;
    };
    /** @brief START carries an immutable per-goal policy snapshot. */
    struct StartRequest {
        /// Accepted task generation.
        std::uint64_t generation;
        /// Original command and user intent.
        proto::ExplorationGoal goal;
        /// Dependencies and policy captured at admission.
        ExplorationTaskBindings bindings;
        /// Cancellation/pause may revoke this gate while the factory waits.
        std::shared_ptr<e::MotionGate> gate;
    };
    /** @brief High-priority command for one accepted generation. */
    struct Command {
        /// START supersedes older commands without affecting a newer request.
        std::uint64_t generation;
        /// Session command or a new START.
        std::variant<e::SessionCommand, StartRequest> value;
    };

    /** @brief Explicit native acquisition dependencies, replaced only while idle. */
    struct NativeInputConfig {
        std::weak_ptr<ExplorationTask> destination;  ///< TaskServer retains actual execution ownership.
        perception::NativeObservationSourceOptions options;  ///< Exact session input policy and channels.
        perception::ObservationIngress::LocalizationLookup localization;  ///< Real acquisition-time pose owner.
        std::chrono::milliseconds timeout;  ///< Steady-clock bound on old Reader retirement.
        ExplorationPacketProcessor processor;  ///< Actual platform operation for this immutable source revision.
    };

    /** @brief Test whether a task generation still owns lifecycle publication. */
    bool Current(std::uint64_t generation) const {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        return !stopping_ && generation == generation_;
    }

    /** @brief A matching generation must also retain current running intent. */
    bool Running(std::uint64_t generation) const {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        return !stopping_ && generation == generation_ &&
               desired_ == TaskLifecycle::kRunning && motion_gate_ &&
               motion_gate_->Enabled();
    }

    /** @brief Dependency replacement cannot race prepared/running native work. */
    void RequireIdleLocked() const {
        if (runtime_busy_ || pending_start_admissions_ || desired_ == TaskLifecycle::kRunning ||
            desired_ == TaskLifecycle::kPaused)
            throw std::logic_error("exploration dependencies require an idle runtime");
    }

    /** @brief Start one owner thread; initialization does not invent services. */
    bool Initialize(std::chrono::milliseconds period) {
        std::lock_guard<std::mutex> join_lock(join_mutex_);
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (worker_started_) return !stopping_;
        stopping_ = false;
        period_ = std::max(std::chrono::milliseconds(1), period);
        worker_ = std::thread([this] { Run(); });
        owner_thread_id_ = worker_.get_id();
        worker_started_ = true;
        return true;
    }

    /** @brief Admit a real runtime while idle or after a terminal generation. */
    bool Configure(ExplorationTaskBindings bindings) {
        if (!bindings.create_session || !bindings.clock) return false;
        bindings.options.driver = e::ExplorationDriver::kBehaviorTree;
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (runtime_busy_ || pending_start_admissions_ || desired_ == TaskLifecycle::kRunning ||
            desired_ == TaskLifecycle::kPaused || motion_fault_ || input_fault_ ||
            native_input_revision_ == std::numeric_limits<std::uint64_t>::max() ||
            (native_input_ && native_input_->options.mode != bindings.options.mode))
            return false;
        bindings_ = std::move(bindings);
        ++native_input_revision_;
        ready_.notify_one();
        return true;
    }

    /** @brief Attach real input; the owner loop is the sole Source creator/caller. */
    bool ConfigureRuntime(const std::shared_ptr<ExplorationTask>& task,
                          const std::shared_ptr<autolink::Node>& node,
                          ExplorationTaskBindings bindings,
                          perception::NativeObservationSourceOptions options,
                          perception::ObservationIngress::LocalizationLookup localization,
                          std::chrono::milliseconds timeout) {
        if (!task || !node || !bindings.create_session || !bindings.clock || !localization ||
            timeout <= std::chrono::milliseconds::zero() || timeout > std::chrono::seconds(30) ||
            options.mode != bindings.options.mode) return false;
        try { perception::ValidateNativeObservationSourceOptions(options); }
        catch (const std::invalid_argument&) { return false; }
        bindings.options.driver = e::ExplorationDriver::kBehaviorTree;
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (node_ != node || input_fault_ || motion_fault_ || runtime_busy_ || pending_start_admissions_ ||
            desired_ == TaskLifecycle::kRunning || desired_ == TaskLifecycle::kPaused ||
            native_input_revision_ == std::numeric_limits<std::uint64_t>::max()) return false;
        bindings_ = std::move(bindings);
        native_input_ = NativeInputConfig{task, std::move(options), std::move(localization), timeout,
            bindings_->process_sensor_packet};
        ++native_input_revision_;
        input_snapshot_.configured = true;
        ready_.notify_one();
        return true;
    }

    /** @brief Attach input separately for compatibility with existing integrations. */
    bool ConfigureInput(const std::shared_ptr<ExplorationTask>& task,
                        perception::NativeObservationSourceOptions options,
                        perception::ObservationIngress::LocalizationLookup localization,
                        std::chrono::milliseconds timeout) {
        if (!task || !localization || timeout <= std::chrono::milliseconds::zero() ||
            timeout > std::chrono::seconds(30)) return false;
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (!node_ || !bindings_ || !bindings_->clock || input_fault_ || motion_fault_ ||
            runtime_busy_ || pending_start_admissions_ || desired_ == TaskLifecycle::kRunning ||
            desired_ == TaskLifecycle::kPaused || options.mode != bindings_->options.mode ||
            native_input_revision_ == std::numeric_limits<std::uint64_t>::max()) return false;
        native_input_ = NativeInputConfig{task, std::move(options), std::move(localization), timeout,
            bindings_->process_sensor_packet};
        ++native_input_revision_;
        input_snapshot_.configured = true;
        ready_.notify_one();
        return true;
    }

    /** @brief Publish actual Source evidence without holding inbox during native locks. */
    void PublishInputSnapshot() {
        const auto source = native_source_ ? native_source_->Snapshot() : last_source_snapshot_;
        const auto completion = native_source_ ? native_source_->ShutdownCompletion() : last_source_completion_;
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        input_snapshot_.source = source;
        input_snapshot_.identity = native_source_identity_;
        input_snapshot_.retiring = native_source_retiring_;
        input_snapshot_.faulted = input_fault_;
        source_completion_ = completion;
    }

    /** @brief Latch failure once; future successful cleanup cannot silently clear it. */
    void InputFault(const char* reason) {
        {
            std::lock_guard<std::mutex> lock(inbox_mutex_);
            if (input_fault_) return;
            input_fault_ = true;
            input_snapshot_.faulted = true;
            input_snapshot_.error = reason;
        }
        throw std::runtime_error(reason);
    }

    /** @brief Reconcile generations without waiting for Reader teardown on this owner.
     * Pause keeps the current source and receiver ordinals. Terminal generations
     * stop input; only generation zero provides pre-START readiness seeding.
     */
    void ReconcileObservationSource() {
        std::optional<NativeInputConfig> config;
        std::shared_ptr<autolink::Node> node;
        perception::ObservationIngress::Clock clock;
        std::optional<ExplorationSensorInputIdentity> wanted;
        std::uint64_t revision;
        bool faulted;
        {
            std::lock_guard<std::mutex> lock(inbox_mutex_);
            config = native_input_;
            node = node_;
            revision = native_input_revision_;
            faulted = input_fault_;
            if (config && bindings_ && !stopping_ && !faulted &&
                (generation_ == 0 || desired_ == TaskLifecycle::kRunning ||
                 desired_ == TaskLifecycle::kPaused)) {
                wanted = ExplorationSensorInputIdentity{instance_, generation_};
                clock = bindings_->clock;
            }
        }
        const auto wall_now = std::chrono::steady_clock::now();
        if (native_source_) {
            const auto snapshot = native_source_->Snapshot();
            if (!native_source_retiring_ &&
                (snapshot.calibration_invalidated || !snapshot.running || snapshot.retirement.requested)) {
                native_source_->Shutdown();
                native_source_retiring_ = true;
                native_retirement_started_ = wall_now;
                PublishInputSnapshot();
                InputFault(snapshot.calibration_invalidated ?
                    "native input calibration changed without a new authority revision" :
                    "native observation source closed unexpectedly");
            }
            const bool same = wanted && native_source_identity_ &&
                wanted->instance == native_source_identity_->instance &&
                wanted->generation == native_source_identity_->generation &&
                revision == native_source_revision_;
            if (!native_source_retiring_ && (faulted || !same)) {
                native_source_->Shutdown();
                native_source_retiring_ = true;
                native_retirement_started_ = wall_now;
            }
            if (native_source_retiring_) {
                PublishInputSnapshot();
                const auto retirement = native_source_->ShutdownCompletion();
                if (retirement.wait_for(std::chrono::milliseconds::zero()) != std::future_status::ready) {
                    if (wall_now - native_retirement_started_ >= native_source_timeout_)
                        InputFault("native Reader retirement timed out; channels remain reserved");
                    return;
                }
                if (!retirement.get()) {
                    InputFault("native Reader retirement failed; automatic replacement disabled");
                    return;
                }
                last_source_snapshot_ = native_source_->Snapshot();
                last_source_completion_ = retirement;
                native_source_.reset();
                native_source_identity_.reset();
                native_source_retiring_ = false;
            }
        }
        if (!native_source_ && wanted && config && !faulted) {
            const auto binding = SensorBinding(config->destination, *wanted, config->processor, revision);
            native_source_identity_ = *wanted;
            native_source_revision_ = revision;
            native_source_timeout_ = config->timeout;
            try {
                native_source_ = std::make_shared<perception::NativeObservationSource>(
                    std::move(node), config->options, config->localization,
                    binding.sink, std::move(clock), binding.loss);
            } catch (...) {
                native_source_identity_.reset();
                PublishInputSnapshot();
                InputFault("native observation source construction failed");
                return;
            }
            if (!native_source_->Start()) {
                native_source_->Shutdown();
                native_source_retiring_ = true;
                native_retirement_started_ = wall_now;
                PublishInputSnapshot();
                InputFault("native observation Reader creation failed");
            }
        }
        PublishInputSnapshot();
    }

    /** @brief Queue a START without blocking ingress on session construction. */
    bool Start(const proto::ExplorationGoal& goal) {
        std::shared_ptr<common::MotionAdmission> admission;
        /** @brief Finish local pending ownership after the inbox lock has been released. */
        struct PendingStart {
            Impl& owner;  ///< Admission owner remains alive for this synchronous call.
            bool armed = false;  ///< Incremented only after domain validation succeeds.
            ~PendingStart() {
                if (!armed) return;
                std::lock_guard<std::mutex> lock(owner.inbox_mutex_);
                --owner.pending_start_admissions_;
                owner.ready_.notify_all();
            }
        } pending{*this};
        std::unique_lock<std::mutex> lock(inbox_mutex_);
        if (stopping_ || !worker_started_ || !bindings_ || motion_fault_ || input_fault_)
            return false;
        if (!std::isfinite(goal.coverage_target()) || goal.coverage_target() < 0 ||
            goal.coverage_target() > 1) return false;
        // Partial-coverage stopping requires a separately versioned policy.
        if (goal.coverage_target() > 0 && goal.coverage_target() < 1) {
            AWARN << "ExplorationTask: partial coverage policy is not configured";
            return false;
        }
        auto bindings = *bindings_;
        if (goal.has_area() && goal.area().points_size()) {
            const auto rectangle = Rectangle(goal.area());
            if (!rectangle) {
                AWARN << "ExplorationTask: polygon requires a polygon-capable coverage policy";
                return false;
            }
            bindings.options.allowed_region = rectangle;
        }
        if (goal.enable_mapping() && !bindings.prepare_mapping) return false;
        if (!goal.map_name().empty()) bindings.options.map_name = goal.map_name();
        if (generation_ == std::numeric_limits<std::uint64_t>::max() ||
            intent_revision_ == std::numeric_limits<std::uint64_t>::max()) return false;
        const auto generation_before_admission = generation_;
        const auto intent_before_admission = intent_revision_;
        const auto factory = motion_admission_factory_;
        ++pending_start_admissions_;
        pending.armed = true;
        lock.unlock();
        if (factory) admission = factory();
        if (factory && (!admission || !admission->Valid())) return false;
        lock.lock();
        if (stopping_ || !worker_started_ || motion_fault_ ||
            generation_ != generation_before_admission || intent_revision_ != intent_before_admission ||
            (admission && !admission->Valid())) return false;
        ++intent_revision_;
        const auto generation = ++generation_;
        if (motion_gate_) motion_gate_->Retire();
        auto gate = std::make_shared<e::MotionGate>(
            admission ? admission->PermitQuery() : e::MotionGate::Permit{});
        gate->SetEnabled(true);
        motion_gate_ = gate;
        runtime_busy_ = true;
        commands_.clear();
        commands_.push_back({generation, StartRequest{generation, goal, std::move(bindings), std::move(gate)}});
        desired_ = TaskLifecycle::kRunning;
        current_goal_ = goal;
        {
            std::lock_guard<std::recursive_mutex> state_lock(owner_.mutex_);
            if (goal.has_header()) {
                owner_.task_id_ = goal.header().task_id();
                owner_.client_id_ = goal.header().client_id();
            } else { owner_.task_id_.clear(); owner_.client_id_.clear(); }
            owner_.lifecycle_ = TaskLifecycle::kRunning;
            status_ = proto::EXPLORATION_STATUS_PLANNING;
            map_name_ = commands_.back().value.index() == 1 ?
                std::get<StartRequest>(commands_.back().value).bindings.options.map_name : "";
            snapshot_.reset();
            owner_.progress_.Clear();
            owner_.progress_.set_detail("waiting for real map and localization input");
        }
        ready_.notify_one();
        if (admission) admission->Complete(true);
        return true;
    }

    /** @brief Validate and queue control transitions against pending intent. */
    bool Control(e::SessionCommand command) {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (stopping_ || !worker_started_) return false;
        if (command == e::SessionCommand::kPause) {
            if (desired_ != TaskLifecycle::kRunning || commands_.size() >= 64) return false;
            desired_ = TaskLifecycle::kPaused;
            if (motion_gate_) motion_gate_->SetEnabled(false);
        } else if (command == e::SessionCommand::kResume) {
            if (desired_ != TaskLifecycle::kPaused || commands_.size() >= 64) return false;
            desired_ = TaskLifecycle::kRunning;
            if (motion_gate_) motion_gate_->SetEnabled(true);
        } else {
            if (desired_ != TaskLifecycle::kRunning && desired_ != TaskLifecycle::kPaused)
                return false;
            desired_ = TaskLifecycle::kCanceled;
            if (motion_gate_) motion_gate_->Retire();
            // Cancellation must remain admissible even with a full control queue.
            commands_.erase(std::remove_if(commands_.begin(), commands_.end(),
                [](const Command& queued) {
                    return std::holds_alternative<e::SessionCommand>(queued.value);
                }), commands_.end());
        }
        if (intent_revision_ != std::numeric_limits<std::uint64_t>::max()) ++intent_revision_;
        commands_.push_back({generation_, command});
        ready_.notify_one();
        return true;
    }

    /** @brief Reject inbox overflow instead of silently dropping evidence. */
    bool Push(Data data) {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (stopping_ || !worker_started_ || inputs_.size() >= 1024) return false;
        if (!std::holds_alternative<e::MapReadView>(data) &&
            !std::holds_alternative<PoseInput>(data) &&
            !std::holds_alternative<PacketInput>(data) &&
            desired_ != TaskLifecycle::kRunning && desired_ != TaskLifecycle::kPaused)
            return false;
        inputs_.push_back({generation_, std::move(data)});
        ready_.notify_one();
        return true;
    }

    /** @brief Capture input identity before expensive decoding or validation. */
    std::optional<ExplorationSensorInputIdentity> InputIdentity() const {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (stopping_ || !worker_started_ || !bindings_) return {};
        return ExplorationSensorInputIdentity{instance_, generation_};
    }

    /** @brief Preserve loss under the caller-held inbox and generation lock. */
    bool ReportLossLocked(perception::SensorKind sensor, std::uint64_t sequence) {
        if (!sequence || (sensor != perception::SensorKind::kLidar &&
            sensor != perception::SensorKind::kRgbd)) return false;
        if (stopping_ || !worker_started_ || !bindings_ ||
            !(perception::Mask(bindings_->options.mode) & perception::Mask(sensor)) ||
            (desired_ != TaskLifecycle::kRunning && desired_ != TaskLifecycle::kPaused)) return false;
        auto& slot = source_losses_[sensor == perception::SensorKind::kLidar ? 0 : 1];
        if (!slot || slot->generation != generation_) slot = SourceLoss{generation_, sensor, sequence, sequence};
        else { slot->first = std::min(slot->first, sequence); slot->last = std::max(slot->last, sequence); }
        ready_.notify_one();
        return true;
    }

    /** @brief Attach explicit rejection only to its original destination. */
    bool ReportLoss(ExplorationSensorInputIdentity identity,
                    perception::SensorKind sensor, std::uint64_t sequence) {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (identity.instance != instance_ || identity.generation != generation_) return false;
        return ReportLossLocked(sensor, sequence);
    }

    /** @brief Admit or record packet loss under the same generation lock. */
    bool PushPacket(ExplorationSensorInputIdentity identity,
        perception::ObservationPacket packet, double now,
        ExplorationPacketProcessor processor = {}, std::uint64_t input_revision = 0) {
        // Every beam is checked without delaying START or cancellation on the inbox lock.
        const bool valid = packet.Valid(now);
        const auto sensor = packet.observation.sensor;
        const auto sequence = packet.observation.sequence;
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (identity.instance != instance_ || identity.generation != generation_ ||
            (input_revision && input_revision != native_input_revision_) ||
            stopping_ || !worker_started_ || !bindings_ ||
            (sensor != perception::SensorKind::kLidar && sensor != perception::SensorKind::kRgbd) ||
            !(perception::Mask(bindings_->options.mode) & perception::Mask(sensor))) return false;
        if (generation_ && desired_ != TaskLifecycle::kRunning && desired_ != TaskLifecycle::kPaused)
            return false;
        if (!valid || inputs_.size() >= 1024) {
            ReportLossLocked(sensor, sequence);
            return false;
        }
        inputs_.push_back({generation_, PacketInput{std::move(packet), now, std::move(processor), input_revision}});
        ready_.notify_one();
        return true;
    }

    /** @brief Obtain a consistent outcome without acquiring the owner inbox. */
    std::optional<e::ExplorationSnapshot> Snapshot() const {
        std::lock_guard<std::recursive_mutex> lock(owner_.mutex_);
        return snapshot_;
    }

    /** @brief Stop admission and wake the owner before joining it. */
    void Shutdown() {
        bool on_owner;
        {
            std::lock_guard<std::mutex> lock(inbox_mutex_);
            stopping_ = true;
            if (motion_gate_) motion_gate_->Retire();
            on_owner = worker_started_ && owner_thread_id_ == std::this_thread::get_id();
            commands_.clear();
            inputs_.clear();
            ready_.notify_all();
        }
        // A dependency may request shutdown from the owner. The outer owner
        // still joins before destruction; self-join would deadlock.
        if (on_owner || callback_instance == instance_) return;
        std::lock_guard<std::mutex> join_lock(join_mutex_);
        if (worker_.joinable()) worker_.join();
        {
            std::lock_guard<std::mutex> lock(inbox_mutex_);
            worker_started_ = false;
            owner_thread_id_ = {};
        }
        std::lock_guard<std::recursive_mutex> state_lock(owner_.mutex_);
        status_ = proto::EXPLORATION_STATUS_IDLE;
        owner_.lifecycle_ = TaskLifecycle::kIdle;
        owner_.progress_.Clear();
        owner_.task_id_.clear();
        owner_.client_id_.clear();
    }

    /** @brief Cancel and destroy runtime objects only on their owner thread. */
    bool StopSession(double now) noexcept {
        bool clean = true;
        if (active_gate_) active_gate_->Retire();
        try {
            if (session_) {
                if (session_->Snapshot().state != e::ExplorationState::kFinished) {
                    if (!session_->PostCommand(e::SessionCommand::kCancel)) clean = false;
                    const auto final = session_->Tick(std::max(now, last_now_));
                    Publish(final, active_generation_);
                }
            }
        } catch (...) { clean = false; }
        try { if (tree_) tree_->Halt(); } catch (...) { clean = false; }
        tree_.reset();
        session_.reset();
        prepared_.reset();
        if (dispatcher_) {
            try { dispatcher_->Shutdown(); } catch (...) { clean = false; }
            if (dispatcher_->Failed()) clean = false;
            dispatcher_.reset();
        }
        active_gate_.reset();
        dispatcher_frame_.clear();
        dispatcher_planner_.clear();
        waiting_.reset();
        waiting_control_.reset();
        mapping_prepared_ = false;
        initialization_resumed_ = false;
        posted_control_.reset();
        deferred_.clear();
        deferred_losses_ = {};
        active_clock_ = {};
        {
            std::lock_guard<std::mutex> lock(inbox_mutex_);
            // A newer accepted START still owns a pending runtime requirement.
            if (active_generation_ == generation_) runtime_busy_ = false;
            if (!clean) motion_fault_ = true;
        }
        return clean;
    }

    /** @brief Read a finite shared clock and reject backwards progression. */
    double Now() {
        const auto now = active_clock_ ? active_clock_() : last_now_;
        if (!std::isfinite(now) || now < last_now_)
            throw std::runtime_error("exploration input clock is not monotonic");
        last_now_ = now;
        return now;
    }

    /** @brief Begin a fresh generation after canceling previous owned work. */
    void Begin(StartRequest request) {
        if (!StopSession(last_now_)) {
            Fail("previous exploration cleanup/cancellation could not be confirmed", request.generation);
            return;
        }
        if (!Current(request.generation)) return;
        active_generation_ = request.generation;
        active_gate_ = request.gate;
        active_clock_ = request.bindings.clock;
        last_now_ = 0;
        const auto now = Now();
        waiting_since_ = now;
        waiting_ = std::move(request);
    }

    /** @brief Seed a new session only from current, matching map/pose evidence. */
    void StartReady(double now) {
        if (!waiting_ || !Running(waiting_->generation)) return;
        if (!mapping_prepared_ && waiting_->goal.enable_mapping()) {
            const auto generation = waiting_->generation;
            const auto prepare = waiting_->bindings.prepare_mapping;
            const auto map_name = waiting_->bindings.options.map_name;
            const bool ready = prepare(map_name);
            // The callback is external and may admit cancel, pause or START.
            // Its result must never initialize a replacement generation.
            if (!Current(generation)) return;
            if (!ready) throw std::runtime_error("real mapping session could not be prepared");
            mapping_prepared_ = true;
            if (!Running(generation)) return;
        }
        const auto& options = waiting_->bindings.options;
        if (!latest_map_ || !latest_pose_ || latest_map_->map_epoch != latest_pose_->epoch ||
            !latest_pose_->support || latest_pose_->stamp > now || latest_map_->source_stamp > now ||
            now - latest_pose_->stamp > options.pose_max_age ||
            now - latest_map_->source_stamp > options.map_max_age) {
            if (now - waiting_since_ >= options.maximum_sensor_wait) {
                throw std::runtime_error("fresh map/localization input unavailable");
            }
            return;
        }
        if (!prepared_) {
            prepared_ = waiting_->bindings.create_session(waiting_->goal, options);
            if (!prepared_ || prepared_->driver() != e::ExplorationDriver::kBehaviorTree ||
                prepared_->Snapshot().state != e::ExplorationState::kIdle)
                throw std::runtime_error("factory must return an unstarted BT-driven session");
        }
        // Keep an unstarted candidate across pause; destroying its service
        // dispatcher here would permanently retire the resumable gate.
        if (!Running(waiting_->generation)) return;
        now = Now();
        if (latest_pose_->stamp > now || latest_map_->source_stamp > now ||
            now - latest_pose_->stamp > options.pose_max_age ||
            now - latest_map_->source_stamp > options.map_max_age) return;
        {
            std::lock_guard<std::mutex> lock(inbox_mutex_);
            if (stopping_ || waiting_->generation != generation_ ||
                desired_ != TaskLifecycle::kRunning || !motion_gate_ ||
                !motion_gate_->Enabled()) return;
            // Admission linearizes with Pause/Cancel/START under this lock.
            // The validated unstarted session has no queued planning, map or
            // navigation work, so Start performs internal initialization only.
            // Factory/clock/native callbacks remain outside the admission lock.
            prepared_->Start(latest_pose_->pose, latest_map_->map_epoch, now);
        }
        auto candidate = std::move(prepared_);
        if (!candidate->PushMap(*latest_map_) ||
            !candidate->PushPose(latest_pose_->pose, latest_pose_->stamp, latest_pose_->epoch,
                                latest_pose_->support))
            throw std::runtime_error("session refused admitted map/localization input");
        auto context = std::make_shared<plugins::exploration::ExplorationBtContext>();
        context->session = candidate;
        context->clock = active_clock_;
        auto tree = std::make_unique<e::ExplorationTree>(context);
        active_options_ = options;
        session_ = std::move(candidate);
        tree_ = std::move(tree);
        waiting_.reset();
        for (auto& input : deferred_) Deliver(std::move(input), now);
        deferred_.clear();
        auto losses = std::move(deferred_losses_);
        deferred_losses_ = {};
        for (const auto& loss : losses) if (loss) DeliverLoss(*loss);
    }

    /** @brief Loss remains attached to its original generation through startup. */
    void DeliverLoss(const SourceLoss& loss) {
        if (loss.generation != active_generation_) return;
        if (session_) {
            session_->RejectSensorInput(loss.sensor, loss.first);
            if (loss.last != loss.first) session_->RejectSensorInput(loss.sensor, loss.last);
        } else if (waiting_) {
            auto& slot = deferred_losses_[loss.sensor == perception::SensorKind::kLidar ? 0 : 1];
            if (!slot) slot = loss;
            else { slot->first = std::min(slot->first, loss.first); slot->last = std::max(slot->last, loss.last); }
        }
    }

    /** @brief Route input to the owner without renewing its evidence timestamp. */
    void Deliver(Input input, double now) {
        if (auto* map = std::get_if<e::MapReadView>(&input.data)) {
            if (!latest_map_ || map->map_epoch != latest_map_->map_epoch ||
                map->source_stamp >= latest_map_->source_stamp) latest_map_ = *map;
            if (session_ && input.generation == active_generation_ && !session_->PushMap(std::move(*map)))
                throw std::runtime_error("session refused an admitted map input");
        } else if (auto* pose = std::get_if<PoseInput>(&input.data)) {
            if ((!latest_map_ || pose->epoch == latest_map_->map_epoch) &&
                (!latest_pose_ || pose->epoch != latest_pose_->epoch ||
                 pose->stamp >= latest_pose_->stamp)) latest_pose_ = *pose;
            if (session_ && input.generation == active_generation_)
                if (!session_->PushPose(pose->pose, pose->stamp, pose->epoch, pose->support))
                    throw std::runtime_error("session refused an admitted pose input");
        } else if (auto* packet = std::get_if<PacketInput>(&input.data)) {
            if (input.generation && input.generation != active_generation_) return;
            if (packet->input_revision) {
                std::lock_guard<std::mutex> lock(inbox_mutex_);
                if (packet->input_revision != native_input_revision_) return;
            }
            const auto processing_time = std::max(now, packet->now);
            if (packet->packet.Valid(processing_time)) {
                if (packet->processor) {
                    auto mapped = packet->processor(input.generation, packet->packet, processing_time);
                    if (input.generation && !Current(input.generation)) return;
                    if (packet->input_revision) {
                        std::lock_guard<std::mutex> lock(inbox_mutex_);
                        if (packet->input_revision != native_input_revision_) return;
                    }
                    if (!mapped || !mapped->grid || mapped->map_epoch != packet->packet.localization.map_epoch ||
                        mapped->frame_id != packet->packet.localization.world_frame ||
                        !std::isfinite(mapped->source_stamp)) {
                        DeliverLoss({input.generation, packet->packet.observation.sensor,
                            packet->packet.observation.sequence, packet->packet.observation.sequence});
                        return;
                    }
                    if (!latest_map_ || mapped->map_epoch != latest_map_->map_epoch ||
                        mapped->source_stamp >= latest_map_->source_stamp) latest_map_ = *mapped;
                    if (session_ && input.generation == active_generation_ && !session_->PushMap(std::move(*mapped)))
                        throw std::runtime_error("session refused actual mapped packet geometry");
                }
                const auto& localization = packet->packet.localization;
                const auto stamp = packet->packet.observation.stamp;
                if ((!latest_map_ || localization.map_epoch == latest_map_->map_epoch) &&
                    (!latest_pose_ || localization.map_epoch != latest_pose_->epoch || stamp >= latest_pose_->stamp))
                    latest_pose_ = PoseInput{{localization.world_from_base.translation.x,
                        localization.world_from_base.translation.y,
                        std::atan2(localization.world_from_base.rotation[3], localization.world_from_base.rotation[0])},
                        stamp, localization.map_epoch, localization.support};
            }
            if (!input.generation || input.generation != active_generation_) return;
            if (!session_) {
                if (waiting_) {
                    if (deferred_.size() >= 1024) {
                        DeliverLoss({input.generation, packet->packet.observation.sensor,
                            packet->packet.observation.sequence, packet->packet.observation.sequence});
                    } else deferred_.push_back(std::move(input));
                }
                return;
            }
            // Session records expected expiry/capacity rejection as source loss.
            session_->PushSensorPacket(std::move(packet->packet), processing_time);
        } else if (input.generation == active_generation_) {
            if (!session_) {
                if (waiting_) {
                    if (deferred_.size() >= 1024)
                        throw std::runtime_error("admitted exploration evidence exceeded deferred capacity");
                    deferred_.push_back(std::move(input));
                }
                return;
            }
            if (auto* observation = std::get_if<ObservationInput>(&input.data)) {
                // Validate at the actual processing time, not an old callback
                // time that would make an expired observation appear current.
                if (!session_->PushObservation(std::move(observation->observation),
                                                std::max(now, observation->now)))
                    throw std::runtime_error("session refused an admitted observation");
            } else if (auto* unit = std::get_if<e::CoverageUnit>(&input.data))
                if (!session_->PushCoverageUnit(std::move(*unit)))
                    throw std::runtime_error("session refused an admitted coverage requirement");
        }
    }

    /** @brief Publish outcome only if this generation still owns the task. */
    void Publish(const e::ExplorationSnapshot& snapshot, std::uint64_t generation) {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (generation != generation_) return;
        if (desired_ == TaskLifecycle::kCanceled &&
            snapshot.result.exploration != e::ExplorationOutcome::kCanceled) return;
        std::lock_guard<std::recursive_mutex> state_lock(owner_.mutex_);
        snapshot_ = snapshot;
        if (snapshot.state == e::ExplorationState::kFinished) {
            if (snapshot.result.exploration == e::ExplorationOutcome::kCanceled) {
                status_ = proto::EXPLORATION_STATUS_CANCELED;
                owner_.lifecycle_ = TaskLifecycle::kCanceled;
            } else if (snapshot.result.Success(active_options_.require_return,
                                                active_options_.require_save)) {
                status_ = proto::EXPLORATION_STATUS_SUCCEEDED;
                owner_.lifecycle_ = TaskLifecycle::kSucceeded;
                owner_.progress_.set_progress(1);
            } else {
                status_ = proto::EXPLORATION_STATUS_FAILED;
                owner_.lifecycle_ = TaskLifecycle::kFailed;
            }
            desired_ = owner_.lifecycle_;
        } else if (snapshot.state == e::ExplorationState::kPaused || desired_ == TaskLifecycle::kPaused) {
            status_ = proto::EXPLORATION_STATUS_PAUSED;
            owner_.lifecycle_ = TaskLifecycle::kPaused;
        } else {
            status_ = snapshot.state == e::ExplorationState::kInitializing ?
                proto::EXPLORATION_STATUS_PLANNING : proto::EXPLORATION_STATUS_EXPLORING;
            owner_.lifecycle_ = TaskLifecycle::kRunning;
        }
        owner_.progress_.set_detail(snapshot.reason);
    }

    /** @brief Report missing/failing real dependencies without forged success. */
    void Fail(const std::string& reason, std::uint64_t generation) {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (generation != generation_) return;
        if (desired_ == TaskLifecycle::kCanceled && !motion_fault_) {
            std::lock_guard<std::recursive_mutex> state_lock(owner_.mutex_);
            status_ = proto::EXPLORATION_STATUS_CANCELED;
            owner_.lifecycle_ = TaskLifecycle::kCanceled;
            owner_.progress_.set_detail("exploration canceled before runtime completion");
            return;
        }
        desired_ = TaskLifecycle::kFailed;
        if (motion_gate_) motion_gate_->Retire();
        std::lock_guard<std::recursive_mutex> state_lock(owner_.mutex_);
        status_ = proto::EXPLORATION_STATUS_FAILED;
        owner_.lifecycle_ = TaskLifecycle::kFailed;
        owner_.progress_.set_detail(reason);
    }

    /**
     * @brief Reconcile current ingress intent before advancing BT actions.
     * @param now Validated time from this owner's configured clock.
     * @return True only while this generation may advance task stages.
     * @details Already extracted control batches can lag Pause/Cancel/START.
     * Monitor a paused session directly, preserving its active BT node and
     * observation phase while consuming bounded input and cancellation work.
     */
    bool ReconcileBeforeTree(double now) {
        if (!session_) return false;
        TaskLifecycle intent;
        bool current;
        {
            std::lock_guard<std::mutex> lock(inbox_mutex_);
            current = !stopping_ && active_generation_ == generation_;
            intent = desired_;
        }
        if (!current || intent == TaskLifecycle::kCanceled || intent == TaskLifecycle::kFailed) {
            const auto generation = active_generation_;
            if (!StopSession(now)) Fail("exploration cancellation could not be confirmed", generation);
            return false;
        }
        if (intent == TaskLifecycle::kPaused) {
            if (posted_control_ != e::SessionCommand::kPause) {
                if (!session_->PostCommand(e::SessionCommand::kPause))
                    throw std::runtime_error("session refused authoritative pause");
                posted_control_ = e::SessionCommand::kPause;
            }
            const auto snapshot = session_->Tick(now);
            Publish(snapshot, active_generation_);
            if (snapshot.state == e::ExplorationState::kFinished) StopSession(now);
            return false;
        }
        if (intent != TaskLifecycle::kRunning) return false;
        if (posted_control_ == e::SessionCommand::kPause ||
            session_->Snapshot().state == e::ExplorationState::kPaused) {
            if (!session_->PostCommand(e::SessionCommand::kResume))
                throw std::runtime_error("session refused authoritative resume");
            posted_control_ = e::SessionCommand::kResume;
            const auto snapshot = session_->Tick(now);
            Publish(snapshot, active_generation_);
            if (snapshot.state == e::ExplorationState::kFinished) {
                StopSession(now);
                return false;
            }
            if (snapshot.state == e::ExplorationState::kPaused) return false;
        }
        // Native readiness, acceptance and execution also check their lease:
        // revocation during an already admitted tree tick cannot authorize motion.
        return Running(active_generation_);
    }

    /** @brief Consume priority commands and bounded data before each tree tick. */
    void Run() {
        for (;;) {
            std::deque<Command> commands;
            std::deque<Input> inputs;
            std::array<std::optional<SourceLoss>, 2> losses;
            bool stopping;
            {
                std::unique_lock<std::mutex> lock(inbox_mutex_);
                ready_.wait_for(lock, period_, [this] {
                    return stopping_ || !commands_.empty() || !inputs_.empty() || source_losses_[0] || source_losses_[1];
                });
                stopping = stopping_;
                commands.swap(commands_);
                losses = std::move(source_losses_);
                source_losses_ = {};
                for (std::size_t i = 0; i < 32 && !inputs_.empty(); ++i) {
                    inputs.push_back(std::move(inputs_.front()));
                    inputs_.pop_front();
                }
            }
            if (stopping) break;
            try {
                for (auto& command : commands) {
                    if (!Current(command.generation)) continue;
                    if (auto* start = std::get_if<StartRequest>(&command.value)) {
                        deferred_.clear();
                        Begin(std::move(*start));
                    } else {
                        const auto control = std::get<e::SessionCommand>(command.value);
                        if (session_) {
                            if (!session_->PostCommand(control))
                                throw std::runtime_error("session refused an admitted lifecycle command");
                            posted_control_ = control;
                        }
                        else if (control == e::SessionCommand::kCancel) {
                            const bool clean = StopSession(last_now_);
                            e::ExplorationSnapshot canceled;
                            canceled.state = e::ExplorationState::kFinished;
                            canceled.result.exploration = e::ExplorationOutcome::kCanceled;
                            canceled.reason = "exploration canceled before inputs became ready";
                            Publish(canceled, command.generation);
                            if (!clean) Fail("exploration cancellation could not be confirmed", command.generation);
                        } else if (waiting_) {
                            waiting_control_ = control;
                            if (control == e::SessionCommand::kResume) initialization_resumed_ = true;
                        }
                    }
                }
                ReconcileObservationSource();
                const auto now = Now();
                if (waiting_ && initialization_resumed_) {
                    waiting_since_ = now;
                    initialization_resumed_ = false;
                }
                for (auto& input : inputs) Deliver(std::move(input), now);
                for (const auto& loss : losses) if (loss) DeliverLoss(*loss);
                StartReady(now);
                if (session_) {
                    if (dispatcher_ && dispatcher_->Failed())
                        throw std::runtime_error("native navigation cancellation could not be confirmed");
                    if (waiting_control_) {
                        if (!session_->PostCommand(*waiting_control_))
                            throw std::runtime_error("session refused pending pause/resume");
                        posted_control_ = waiting_control_;
                        waiting_control_.reset();
                    }
                    if (!ReconcileBeforeTree(now)) continue;
                    const auto snapshot = tree_->Tick();
                    Publish(snapshot, active_generation_);
                    if (snapshot.state == e::ExplorationState::kFinished) StopSession(now);
                }
            } catch (const std::exception& exception) {
                try { StopSession(last_now_); } catch (...) {}
                std::uint64_t failure_generation;
                { std::lock_guard<std::mutex> lock(inbox_mutex_);
                  failure_generation = input_fault_ ? generation_ : active_generation_; }
                Fail(exception.what(), failure_generation);
            } catch (...) {
                try { StopSession(last_now_); } catch (...) {}
                Fail("exploration dependency threw a nonstandard exception", active_generation_);
            }
        }
        try { StopSession(last_now_); } catch (...) {}
        if (native_source_) {
            native_source_->Shutdown();
            native_source_retiring_ = true;
        }
        PublishInputSnapshot();
    }

    /// Native lifecycle and result owner; outlives the worker.
    ExplorationTask& owner_;
    /// Unique native request scope for the task object.
    const std::uint64_t instance_;
    std::array<std::optional<SourceLoss>, 2> source_losses_;  ///< Inbox-locked loss slots independent from data capacity.
    std::array<std::optional<SourceLoss>, 2> deferred_losses_;  ///< Owner-only loss accumulated before Session starts.
    /// Serializes inbox admission, configuration and generation changes.
    mutable std::mutex inbox_mutex_;
    /// Serializes external worker joins; never held by feedback/tick.
    std::mutex join_mutex_;
    /// Wakes the owner for commands, data or shutdown.
    std::condition_variable ready_;
    /// Native task loop with sole mutable Session/Tree access.
    std::thread worker_;
    /// Cached under inbox_mutex_; never reads a thread concurrently with join.
    std::thread::id owner_thread_id_;
    /// Admission checks this field instead of thread::joinable.
    bool worker_started_ = false;
    /// Prepared/active generation must drain before dependency replacement.
    bool runtime_busy_ = false;
    std::size_t pending_start_admissions_ = 0;  ///< Inbox-protected starts awaiting scheduler ownership.
    std::uint64_t intent_revision_ = 0;  ///< START and control transitions invalidate late admission callbacks.
    common::MotionAdmission::Factory motion_admission_factory_;  ///< Invoked outside inbox/lifecycle locks.
    /// Unconfirmed old cancellation prevents replacement by another START.
    bool motion_fault_ = false;
    /// Current generation permission, revoked directly by admission commands.
    std::shared_ptr<e::MotionGate> motion_gate_;
    /// Native scheduler tick period, copied before worker creation.
    std::chrono::milliseconds period_{100};
    /// Blocks all new command/data admission during shutdown.
    bool stopping_ = false;
    /// Current accepted task generation, protected by inbox_mutex_.
    std::uint64_t generation_ = 0;
    /// Last admitted control intent, including commands not yet consumed.
    TaskLifecycle desired_ = TaskLifecycle::kIdle;
    /// Immutable configured dependencies, replaced only while inactive.
    std::optional<ExplorationTaskBindings> bindings_;
    std::optional<NativeInputConfig> native_input_;  ///< Inbox-locked configuration, with weak task ownership.
    std::uint64_t native_input_revision_ = 0;  ///< Configuration/node/clock changes require actual retirement.
    bool input_fault_ = false;  ///< Inbox-locked failure latch checked by START/configuration.
    ExplorationObservationSourceSnapshot input_snapshot_;  ///< Inbox-locked public input lifecycle evidence.
    std::shared_future<bool> source_completion_;  ///< Stable public handle to the latest cleanup operation.
    std::shared_ptr<perception::NativeObservationSource> native_source_;  ///< Owner-thread-only real Reader source.
    std::optional<ExplorationSensorInputIdentity> native_source_identity_;  ///< Immutable callback destination.
    std::uint64_t native_source_revision_ = 0;  ///< Configuration used to construct this Source.
    bool native_source_retiring_ = false;  ///< Owner requested revocation; does not mean cleanup succeeded.
    std::chrono::steady_clock::time_point native_retirement_started_;  ///< Independent of paused task time.
    std::chrono::milliseconds native_source_timeout_{5000};  ///< Timeout of the retiring Source, not the replacement.
    perception::NativeObservationSourceSnapshot last_source_snapshot_;  ///< Preserve completed cleanup evidence.
    std::shared_future<bool> last_source_completion_;  ///< Keep success/failure observable after Source release.
    /// Current original goal for intent-preserving area updates.
    proto::ExplorationGoal current_goal_;
    /// High-priority commands bounded independently of observation backlog.
    std::deque<Command> commands_;
    /// Owned bounded producer events.
    std::deque<Input> inputs_;
    /// Native node compatibility injection; no competing waypoint readers.
    std::shared_ptr<autolink::Node> node_;
    /// Dispatcher installed by TaskServer.
    SubmitNavigation submit_navigation_;
    /// Cooperative native START; legacy submission never backs new services.
    GuardedSubmitNavigation guarded_submit_navigation_;
    /// Legacy mapping dispatcher does not supply a save/fusion receipt.
    SubmitMapping submit_mapping_;
    /// Identity-aware native result probe.
    GetVersionedNavigationResult navigation_result_;
    /// Conditional cancellation of the current owned navigation task.
    CancelNavigationRequest cancel_navigation_;
    /// Native feedback status, protected by owner_.mutex_.
    proto::ExplorationStatus status_ = proto::EXPLORATION_STATUS_IDLE;
    /// Map identity associated with accepted task policy, same protection.
    std::string map_name_;
    /// Consistent algorithm outcome, protected by owner_.mutex_.
    std::optional<e::ExplorationSnapshot> snapshot_;
    /// Owner-only generation of the active runtime.
    std::uint64_t active_generation_ = 0;
    /// Owner-only pending START awaiting real seed evidence.
    std::optional<StartRequest> waiting_;
    /// Owner-only pause/resume intent before runtime creation.
    std::optional<e::SessionCommand> waiting_control_;
    /// This generation completed its real mapping precondition before Start.
    bool mapping_prepared_ = false;
    /// Resume grants a fresh bounded seed wait rather than charging paused time.
    bool initialization_resumed_ = false;
    /// Last control queued to Session, reconciled with the latest ingress intent.
    std::optional<e::SessionCommand> posted_control_;
    /// Input requirements retained while a START waits for real map/pose.
    std::deque<Input> deferred_;
    /// Latest owned map with its original source stamp and alignment.
    std::optional<e::MapReadView> latest_map_;
    /// Latest pose used for start seeding; never manufactures localization.
    std::optional<PoseInput> latest_pose_;
    /// Owner-only shared clock copied from accepted START.
    std::function<double()> active_clock_;
    /// Last accepted monotonic owner time.
    double last_now_ = 0;
    /// Start-wait time, not refreshed by repeated messages.
    double waiting_since_ = 0;
    /// Actual options used to interpret the terminal three-part result.
    e::ExplorationSessionOptions active_options_;
    /// Session created, started and destroyed on this owner.
    std::shared_ptr<e::ExplorationSession> session_;
    /// A paused factory result remains unstarted until running intent returns.
    std::shared_ptr<e::ExplorationSession> prepared_;
    /// Owner generation gate; may differ from a newer admitted generation.
    std::shared_ptr<e::MotionGate> active_gate_;
    /// One asynchronous native dispatcher; closed before native dependencies.
    std::shared_ptr<e::NavigationDispatcher> dispatcher_;
    /// One dispatcher cannot silently change its native reference frame.
    std::string dispatcher_frame_;
    /// Planner selection is fixed for this dispatcher generation.
    std::string dispatcher_planner_;
    /// Real native tree, never used by producer callbacks.
    std::unique_ptr<e::ExplorationTree> tree_;
};

ExplorationTask::ExplorationTask() : impl_(std::make_unique<Impl>(*this)) {}
ExplorationTask::~ExplorationTask() { Shutdown(); }
::automsgs::msgs::vehicle_msgs::RobotTaskType ExplorationTask::GetTaskType() const {
    return ::automsgs::msgs::vehicle_msgs::RobotTaskType::ROBOT_TASK_EXPLORATION;
}
void ExplorationTask::SetNode(std::shared_ptr<autolink::Node> node) {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    impl_->RequireIdleLocked();
    if (impl_->native_input_revision_ == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("exploration input configuration revision exhausted");
    impl_->node_ = std::move(node);
    ++impl_->native_input_revision_;
    impl_->ready_.notify_one();
}
void ExplorationTask::SetSubmitNavigation(SubmitNavigation submit) {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    impl_->RequireIdleLocked(); impl_->submit_navigation_ = std::move(submit);
}
void ExplorationTask::SetGuardedSubmitNavigation(GuardedSubmitNavigation submit) {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    impl_->RequireIdleLocked(); impl_->guarded_submit_navigation_ = std::move(submit);
}
void ExplorationTask::SetMotionAdmission(common::MotionAdmission::Factory factory) {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    impl_->RequireIdleLocked();
    impl_->motion_admission_factory_ = std::move(factory);
}
bool ExplorationTask::MotionDrained() const {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    return !impl_->runtime_busy_ && !impl_->pending_start_admissions_ && !impl_->motion_fault_;
}
void ExplorationTask::SetSubmitMapping(SubmitMapping submit) {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    impl_->RequireIdleLocked(); impl_->submit_mapping_ = std::move(submit);
}
void ExplorationTask::SetNavigationProbe(IsNavigationActive, GetNavigationResult) {
    // The compatibility surface deliberately cannot fabricate versioned results.
}
void ExplorationTask::SetVersionedNavigationProbe(GetVersionedNavigationResult result,
                                                 CancelNavigationRequest cancel) {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    impl_->RequireIdleLocked();
    impl_->navigation_result_ = std::move(result); impl_->cancel_navigation_ = std::move(cancel);
}
bool ExplorationTask::ConfigureSession(ExplorationTaskBindings bindings) {
    return impl_->Configure(std::move(bindings));
}
bool ExplorationTask::ConfigureNativeRuntime(const std::shared_ptr<ExplorationTask>& task,
    const std::shared_ptr<autolink::Node>& node, ExplorationTaskBindings bindings,
    perception::NativeObservationSourceOptions input,
    perception::ObservationIngress::LocalizationLookup localization,
    std::chrono::milliseconds retirement_timeout) {
    return task.get() == this && impl_->ConfigureRuntime(task, node, std::move(bindings),
        std::move(input), std::move(localization), retirement_timeout);
}
bool ExplorationTask::ConfigureObservationSource(const std::shared_ptr<ExplorationTask>& task,
    perception::NativeObservationSourceOptions options,
    perception::ObservationIngress::LocalizationLookup localization,
    std::chrono::milliseconds retirement_timeout) {
    return task && task->impl_->ConfigureInput(task, std::move(options),
                                              std::move(localization), retirement_timeout);
}
ExplorationObservationSourceSnapshot ExplorationTask::GetObservationSourceSnapshot() const {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    return impl_->input_snapshot_;
}
std::shared_future<bool> ExplorationTask::ObservationSourceShutdownCompletion() const {
    std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
    return impl_->source_completion_;
}
e::ExplorationServices ExplorationTask::NavigationServices(
    const std::string& frame_id, const std::string& planner_id, const std::string& controller_id,
    const std::string& navigation_tree) const {
    GuardedSubmitNavigation submit;
    GetVersionedNavigationResult result;
    CancelNavigationRequest cancel;
    std::shared_ptr<e::MotionGate> gate;
    std::string prefix;
    {
        std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
        if (std::this_thread::get_id() != impl_->owner_thread_id_ ||
            impl_->active_generation_ != impl_->generation_ || impl_->stopping_ ||
            (impl_->desired_ != TaskLifecycle::kRunning &&
             impl_->desired_ != TaskLifecycle::kPaused))
            throw std::logic_error("navigation services require the current factory owner");
        submit = impl_->guarded_submit_navigation_;
        result = impl_->navigation_result_;
        cancel = impl_->cancel_navigation_;
        gate = impl_->motion_gate_;
        prefix = "exploration/" + ProcessIdentity() + "/" +
                 std::to_string(impl_->instance_) + "/" +
                 std::to_string(impl_->generation_) + "/";
    }
    if (frame_id.empty() || planner_id.empty() || !gate || !submit || !result || !cancel)
        throw std::logic_error("exploration requires cooperative versioned native services");
    if (!impl_->dispatcher_) {
        const auto instance = impl_->instance_;
        impl_->dispatcher_ = std::make_shared<e::NavigationDispatcher>(
            gate, prefix,
            [submit, frame_id, planner_id, controller_id, navigation_tree, instance](
                const e::NavigationDispatchRequest& request,
                const e::MotionGate::Permit& permit) {
                NativeCallbackScope scope(instance);
                proto::NavigationGoal goal;
                goal.mutable_header()->set_task_id(request.task_id);
                goal.mutable_header()->set_client_id("exploration");
                goal.set_command(proto::NAV_CMD_START);
                goal.set_mode(proto::NAV_MODE_SINGLE_POSE);
                goal.mutable_plugins()->set_planner_id(planner_id);
                if (!controller_id.empty()) goal.mutable_plugins()->set_controller_id(controller_id);
                if (!navigation_tree.empty()) goal.mutable_plugins()->set_behavior_tree(navigation_tree);
                auto* target = goal.add_goals();
                target->mutable_header()->set_frame_id(frame_id);
                target->mutable_pose()->mutable_position()->set_x(request.pose.x);
                target->mutable_pose()->mutable_position()->set_y(request.pose.y);
                target->mutable_pose()->mutable_orientation()->set_z(std::sin(request.pose.yaw / 2));
                target->mutable_pose()->mutable_orientation()->set_w(std::cos(request.pose.yaw / 2));
                return submit(goal, permit);
            },
            [result, instance](const std::string& id) -> std::optional<bool> {
                NativeCallbackScope scope(instance);
                proto::NavigationResult reply;
                if (!result(id, &reply)) return {};
                return reply.final_status() == proto::NAV_STATUS_SUCCEEDED;
            },
            [cancel, instance](const std::string& id) {
                NativeCallbackScope scope(instance);
                // False denotes no matching native request; exceptions mean
                // the backend could not confirm stopping this accepted ID.
                cancel(id);
            });
        impl_->dispatcher_frame_ = frame_id;
        impl_->dispatcher_planner_ = planner_id;
    } else if (impl_->dispatcher_frame_ != frame_id ||
               impl_->dispatcher_planner_ != planner_id) {
        throw std::logic_error("one exploration dispatcher has one native frame/planner");
    }
    const auto dispatcher = impl_->dispatcher_;
    e::ExplorationServices services;
    services.submit_navigation = [dispatcher](const e::Pose2d& pose,
                                              const e::RequestVersion& version) {
        return dispatcher->Enqueue(pose, version);
    };
    services.poll_navigation = [dispatcher] { return dispatcher->Poll(); };
    services.cancel_navigation = [dispatcher](const e::RequestVersion& version) {
        dispatcher->CancelRequest(version);
    };
    return services;
}
bool ExplorationTask::PushMap(e::MapReadView map) {
    if (!map.grid || !map.map_epoch || map.frame_id.empty() || !std::isfinite(map.source_stamp))
        return false;
    return impl_->Push(std::move(map));
}
bool ExplorationTask::PushPose(e::Pose2d pose, double stamp, std::uint64_t epoch,
                              perception::SensorMask support) {
    if (!autonomy::navigation::IsFinite(pose) || !std::isfinite(stamp) || !epoch || !support) return false;
    return impl_->Push(Impl::PoseInput{pose, stamp, epoch, support});
}
bool ExplorationTask::PushObservation(perception::SensorObservation observation, double now) {
    if (!std::isfinite(now)) return false;
    return impl_->Push(Impl::ObservationInput{std::move(observation), now});
}
bool ExplorationTask::PushSensorPacket(perception::ObservationPacket packet, double now) {
    const auto identity = impl_->InputIdentity();
    return identity && impl_->PushPacket(*identity, std::move(packet), now);
}
std::optional<ExplorationSensorInputIdentity> ExplorationTask::SensorInputIdentity() const {
    return impl_->InputIdentity();
}
bool ExplorationTask::PushSensorPacket(ExplorationSensorInputIdentity identity,
                                      perception::ObservationPacket packet, double now) {
    return impl_->PushPacket(identity, std::move(packet), now);
}
bool ExplorationTask::PushNativeSensorPacket(ExplorationSensorInputIdentity identity,
    perception::ObservationPacket packet, double now,
    ExplorationPacketProcessor processor, std::uint64_t input_revision) {
    if (!input_revision) return false;
    return impl_->PushPacket(identity, std::move(packet), now, std::move(processor), input_revision);
}
bool ExplorationTask::ReportSensorLoss(perception::SensorKind sensor, std::uint64_t sequence) {
    const auto identity = impl_->InputIdentity();
    return identity && impl_->ReportLoss(*identity, sensor, sequence);
}
bool ExplorationTask::ReportSensorLoss(ExplorationSensorInputIdentity identity,
                                      perception::SensorKind sensor, std::uint64_t sequence) {
    return impl_->ReportLoss(identity, sensor, sequence);
}
std::optional<ExplorationSensorInputBinding> ExplorationTask::BindSensorInput(
    const std::shared_ptr<ExplorationTask>& task) {
    if (!task) return {};
    const auto identity = task->SensorInputIdentity();
    if (!identity) return {};
    return SensorBinding(task, *identity);
}
bool ExplorationTask::PushCoverageUnit(e::CoverageUnit unit) { return impl_->Push(std::move(unit)); }
std::optional<e::ExplorationSnapshot> ExplorationTask::GetExplorationSnapshot() const {
    return impl_->Snapshot();
}
bool ExplorationTask::SubmitGoal(const proto::ExplorationGoal& goal) { return OnGoal(goal); }
void ExplorationTask::Shutdown() { impl_->Shutdown(); }
bool ExplorationTask::Cancel() { return impl_->Control(e::SessionCommand::kCancel); }
bool ExplorationTask::Pause() { return impl_->Control(e::SessionCommand::kPause); }
bool ExplorationTask::Resume() { return impl_->Control(e::SessionCommand::kResume); }
bool ExplorationTask::OnInitialize(const proto::TaskServerOptions& options) {
    return impl_->Initialize(std::chrono::milliseconds(
        options.scheduler().feedback_period_ms() > 0 ? options.scheduler().feedback_period_ms() : 100));
}
bool ExplorationTask::OnGoal(const proto::ExplorationGoal& goal) {
    switch (goal.command()) {
        case proto::EXPLORATION_CMD_START: return impl_->Start(goal);
        case proto::EXPLORATION_CMD_PAUSE: return Pause();
        case proto::EXPLORATION_CMD_RESUME: return Resume();
        case proto::EXPLORATION_CMD_CANCEL:
        case proto::EXPLORATION_CMD_STOP: return Cancel();
        case proto::EXPLORATION_CMD_SET_AREA: {
            proto::ExplorationGoal changed;
            {
                std::lock_guard<std::mutex> lock(impl_->inbox_mutex_);
                if (impl_->desired_ != TaskLifecycle::kRunning && impl_->desired_ != TaskLifecycle::kPaused)
                    return false;
                changed = impl_->current_goal_;
            }
            *changed.mutable_area() = goal.area();
            changed.set_command(proto::EXPLORATION_CMD_START);
            return impl_->Start(changed);
        }
        case proto::EXPLORATION_CMD_SAVE_MAP:
            AWARN << "ExplorationTask: checkpoint saving needs a real versioned mapping service";
            return false;
        default: return false;
    }
}
void ExplorationTask::FillFeedback(proto::ExplorationFeedback* feedback) const {
    feedback->set_status(impl_->status_);
    *feedback->mutable_progress() = progress_;
    feedback->set_exploration_progress(progress_.progress());
    feedback->set_map_name(impl_->map_name_);
}
void ExplorationTask::FillResult(proto::ExplorationResult* result) const {
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(impl_->status_);
    result->set_map_name(impl_->map_name_);
}
}  // namespace task
}  // namespace autonomy
