/* Copyright 2026 The Openbot Authors. */
/** @file navigation_dispatcher.cpp @brief Revocable native navigation dispatch. */
#include "autonomy/task/exploration/integration/navigation_dispatcher.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace autonomy::task::exploration {

/** @brief Revision changes and permission publication share one short lock. */
struct MotionGate::State {
    explicit State(Permit parent) : parent(std::move(parent)) {}  ///< Capture a bounded external permit.
    const Permit parent;  ///< Immutable scheduler ownership; queried outside the local gate mutex.
    std::atomic<bool> denied{false};  ///< External denial is permanent, including across resume.
    /** @return External eligibility; false/exception permanently retires outstanding leases. */
    bool Allowed() {
        if (denied.load()) return false;
        try { if (!parent || parent()) return !denied.load(); } catch (...) {}
        denied.store(true);
        return false;
    }
    /// Protects all gate fields; no external callback runs while locked.
    mutable std::mutex mutex;
    /// Current lease revision; exhaustion permanently retires the gate.
    std::uint64_t revision = 0;
    /// New leases are allowed only while the task runs.
    bool enabled = false;
    /// A retired task can never authorize another START.
    bool retired = false;
};

MotionGate::MotionGate(Permit parent) : state_(std::make_shared<State>(std::move(parent))) {}
void MotionGate::SetEnabled(bool enabled) {
    if (enabled && !state_->Allowed()) return;
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->retired || state_->enabled == enabled) return;
    if (state_->revision == std::numeric_limits<std::uint64_t>::max()) {
        state_->retired = true;
        state_->enabled = false;
        return;
    }
    ++state_->revision;
    state_->enabled = enabled;
}
void MotionGate::Retire() {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->retired = true;
    state_->enabled = false;
}
MotionGate::Permit MotionGate::Lease() const {
    std::uint64_t revision;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        revision = state_->revision;
    }
    return [state = state_, revision] {
        if (!state->Allowed()) return false;
        std::lock_guard<std::mutex> lock(state->mutex);
        return !state->retired && state->enabled && state->revision == revision;
    };
}
bool MotionGate::Enabled() const {
    if (!state_->Allowed()) return false;
    std::lock_guard<std::mutex> lock(state_->mutex);
    return !state_->retired && state_->enabled;
}

/** @brief Queue state is shared with the worker, but never with a Session. */
class NavigationDispatcher::Impl {
public:
    /** @brief A queued request keeps its own irrevocable cancellation token. */
    struct Job {
        /// Immutable native request and algorithm version.
        NavigationDispatchRequest request;
        /// A cancelled job stays cancelled even after generation resume.
        std::shared_ptr<std::atomic<bool>> live =
            std::make_shared<std::atomic<bool>>(true);
        /// Movement lease captured at admission, not at a later dispatch.
        MotionGate::Permit permit;
        /// Worker reserved submission; protected by mutex.
        bool dispatched = false;
        /// Native acceptance handoff finished; protected by mutex.
        bool submission_finished = false;
        /// Native task accepted this ID; protected by mutex.
        bool accepted = false;
        /// Cancellation has one owner even when submission throws after accept.
        bool cancellation_queued = false;
    };

    /** @brief Validate immutable callbacks before starting the worker. */
    Impl(std::shared_ptr<MotionGate> gate, std::string prefix,
         Submit submit, Result result, Cancel cancel)
        : gate_(std::move(gate)), prefix_(std::move(prefix)),
          submit_(std::move(submit)), result_(std::move(result)),
          cancel_(std::move(cancel)) {
        if (!gate_ || prefix_.empty() || !submit_ || !result_ || !cancel_)
            throw std::invalid_argument("navigation dispatcher needs guarded services");
        worker_ = std::thread([this] { Run(); });
        worker_id_ = worker_.get_id();
    }

    /**
     * @brief Assign native cancellation to the serial worker exactly once.
     * @param job Accepted or ambiguously accepted request to retire.
     * @pre mutex_ is held. The native callback never executes under this lock.
     */
    void QueueCancellation(const std::shared_ptr<Job>& job) {
        if (job->cancellation_queued) return;
        job->cancellation_queued = true;
        job->accepted = false;
        cancellations_.push_back(job);
        ready_.notify_one();
    }

    /** @brief An unconfirmed cancellation permanently disables this generation. */
    void CancellationFailed() {
        std::lock_guard<std::mutex> lock(mutex_);
        failed_ = true;
        gate_->Retire();
    }

    /** @brief Serial worker cancels old accepted IDs before dispatching new IDs. */
    void Run() noexcept {
        for (;;) {
            std::shared_ptr<Job> job;
            bool cancellation = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] {
                    return stopping_ || !cancellations_.empty() ||
                           (pending_ && !pending_->dispatched);
                });
                if (!cancellations_.empty()) {
                    job = std::move(cancellations_.front());
                    cancellations_.pop_front();
                    cancellation = true;
                } else if (pending_ && !pending_->dispatched && !stopping_) {
                    job = pending_;
                    // Reserve this job before releasing the queue lock.
                    job->dispatched = true;
                } else if (stopping_) {
                    break;
                }
                processing_ = true;
            }
            if (!job) continue;
            if (cancellation) {
                try { cancel_(job->request.task_id); }
                catch (...) { CancellationFailed(); }
            } else {
                bool accepted = false;
                bool ambiguous_acceptance = false;
                try {
                    if (job->permit()) accepted = submit_(job->request, job->permit);
                } catch (...) {
                    ambiguous_acceptance = true;
                }
                // CancelRequest uses this same lock. Either it sees accepted
                // and queues cancellation, or this handoff sees its revocation
                // and queues cancellation. Never cache revoked outside the lock.
                std::lock_guard<std::mutex> lock(mutex_);
                const bool revoked = stopping_ || failed_ || pending_ != job ||
                                     !job->permit();
                job->submission_finished = true;
                job->accepted = accepted && !revoked;
                if (ambiguous_acceptance || (accepted && revoked))
                    QueueCancellation(job);
                // A rejected current request remains available to Poll as a
                // failure. Explicitly cancelled/replaced requests have no reply.
                if (pending_ == job && revoked) pending_.reset();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                processing_ = false;
                ready_.notify_all();
            }
        }
    }

    /// Shared generation authorization; it survives task object destruction.
    std::shared_ptr<MotionGate> gate_;
    /// Native identity namespace includes a process boot identity.
    const std::string prefix_;
    /// Cooperative native callbacks, immutable until worker destruction.
    const Submit submit_;
    const Result result_;
    const Cancel cancel_;
    /// Protects admission, pending status and cancellation queue.
    mutable std::mutex mutex_;
    /// External joins serialize independently of callbacks and queue locks.
    std::mutex join_mutex_;
    /// Wakes the serial native worker for enqueue/cancel/shutdown.
    std::condition_variable ready_;
    /// One currently visible request; old dispatch jobs remain owned by worker.
    std::shared_ptr<Job> pending_;
    /// Old accepted IDs awaiting cancellation, processed before a new START.
    std::deque<std::shared_ptr<Job>> cancellations_;
    /// Prevents further enqueue after shutdown begins.
    bool stopping_ = false;
    /// An ambiguous cancellation failure prevents additional motion.
    bool failed_ = false;
    /// Native submit/cancel callback is currently running.
    bool processing_ = false;
    /// Joined only under join_mutex_; callback state never reads this object.
    std::thread worker_;
    /// Immutable ID avoids racing get_id with a concurrent join.
    std::thread::id worker_id_;
};

NavigationDispatcher::NavigationDispatcher(std::shared_ptr<MotionGate> gate,
    std::string prefix, Submit submit, Result result, Cancel cancel)
    : impl_(std::make_unique<Impl>(std::move(gate), std::move(prefix),
          std::move(submit), std::move(result), std::move(cancel))) {}
NavigationDispatcher::~NavigationDispatcher() { Shutdown(); }

bool NavigationDispatcher::Enqueue(const Pose2d& pose,
                                    const RequestVersion& version) {
    if (!autonomy::navigation::IsFinite(pose) || !version.session_id ||
        !version.request_id) return false;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (impl_->stopping_ || impl_->failed_ || !impl_->gate_->Enabled()) return false;
    if (impl_->pending_)
        return impl_->pending_->request.version.SameRequest(version) &&
               impl_->pending_->live->load() &&
               impl_->pending_->request.pose.x == pose.x &&
               impl_->pending_->request.pose.y == pose.y &&
               impl_->pending_->request.pose.yaw == pose.yaw;
    auto job = std::make_shared<Impl::Job>();
    job->request = {impl_->prefix_ + std::to_string(version.session_id) + "/" +
                   std::to_string(version.request_id), pose, version};
    auto lease = impl_->gate_->Lease();
    job->permit = [lease = std::move(lease), live = job->live] {
        return live->load() && lease();
    };
    impl_->pending_ = std::move(job);
    impl_->ready_.notify_one();
    return true;
}

std::optional<NavigationReply> NavigationDispatcher::Poll() {
    std::shared_ptr<Impl::Job> job;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex_);
        job = impl_->pending_;
        if (!job || !job->submission_finished || !job->live->load()) return {};
        if (!job->permit()) {
            job->live->store(false);
            if (job->accepted) impl_->QueueCancellation(job);
            impl_->pending_.reset();
            return {};
        }
        if (!job->accepted) {
            impl_->pending_.reset();
            return NavigationReply{job->request.version, false};
        }
    }
    std::optional<bool> result;
    try { result = impl_->result_(job->request.task_id); }
    catch (...) {
        std::lock_guard<std::mutex> lock(impl_->mutex_);
        if (impl_->pending_ != job) return {};
        job->live->store(false);
        impl_->QueueCancellation(job);
        impl_->pending_.reset();
        return NavigationReply{job->request.version, false};
    }
    if (!result) return {};
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (impl_->pending_ != job) return {};
    if (!job->permit()) {
        job->live->store(false);
        impl_->QueueCancellation(job);
        impl_->pending_.reset();
        return {};
    }
    impl_->pending_.reset();
    return NavigationReply{job->request.version, *result};
}

void NavigationDispatcher::CancelRequest(const RequestVersion& version) {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    const auto job = impl_->pending_;
    if (!job || !job->request.version.SameRequest(version)) return;
    job->live->store(false);
    if (job->accepted) impl_->QueueCancellation(job);
    impl_->pending_.reset();
    impl_->ready_.notify_one();
}
bool NavigationDispatcher::HasInFlight() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    return impl_->pending_ || impl_->processing_ || !impl_->cancellations_.empty();
}
bool NavigationDispatcher::Failed() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    return impl_->failed_;
}
void NavigationDispatcher::Shutdown() {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex_);
        if (!impl_->stopping_) {
            impl_->stopping_ = true;
            impl_->gate_->Retire();
            if (impl_->pending_) {
                impl_->pending_->live->store(false);
                if (impl_->pending_->accepted)
                    impl_->QueueCancellation(impl_->pending_);
                impl_->pending_.reset();
            }
            impl_->ready_.notify_all();
        }
    }
    // A native callback may request shutdown while an external caller joins.
    // Check the immutable ID before waiting on join_mutex_.
    if (impl_->worker_id_ == std::this_thread::get_id()) return;
    std::lock_guard<std::mutex> join_lock(impl_->join_mutex_);
    if (impl_->worker_.joinable()) impl_->worker_.join();
}

}  // namespace autonomy::task::exploration
