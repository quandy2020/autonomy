/* Copyright 2026 The Openbot Authors. */
/** @file scheduler.cpp @brief Revocable task ownership and delegated navigation admission. */
#include "autonomy/task/scheduler/scheduler.hpp"

#include <algorithm>
#include <condition_variable>
#include <stdexcept>
#include <unordered_set>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/exploration/exploration.hpp"
#include "autonomy/task/navigation/navigation.hpp"

namespace autonomy::task {

/**
 * @brief Shared scheduler lifetime, independent of the public host object.
 * @details No task virtual function, transport operation or worker join runs
 * under mutex. Permit callbacks read only immutable handles and atomics.
 * A terminal lifecycle is insufficient to release a native movement owner:
 * pending admission and actual domain cleanup must also be drained.
 */
class TaskScheduler::State : public std::enable_shared_from_this<State> {
public:
    using Admission = common::MotionAdmission;  ///< Internal handle; wire messages remain automsgs.
    /** @brief One registration is unavailable until configuration has completed. */
    struct Entry {
        TaskPointer task;  ///< Strong registry ownership.
        RobotTaskType type;  ///< Cached outside the scheduler lock.
        bool ready = false;  ///< Initialization and hook installation both succeeded.
        bool hooks = false;  ///< Hooks are installed once, independently of registration order.
    };
    /** @brief Immutable execution generation; retirement is irreversible. */
    struct Lease {
        std::weak_ptr<TaskInterface> task;  ///< Domain lifetime belongs to registry/caller.
        RobotTaskType type;  ///< Domain identity, captured before locking.
        std::atomic<bool> retired{false};  ///< Permit queries never acquire the scheduler lock.
        std::size_t pending = 0;  ///< Admissions whose synchronous handoff has not completed.
        bool preliminary = false;  ///< RequestActivation to SubmitGoal reservation.
        std::uint64_t revision = 0;  ///< Invalidates a stale outside-lock drain snapshot.
        std::vector<std::weak_ptr<Lease>> children;  ///< Avoid unbounded retained exploration history.
    };

    explicit State(proto::SchedulerOptions options) { ConfigureOptions(options); }

    /** @brief Copy options only while admission is closed and no task is active. */
    void ConfigureOptions(const proto::SchedulerOptions& options) {
        exclusive = options.exclusive_navigation_tasks();
        preemption = !options.has_allow_preemption() || options.allow_preemption();
        period = std::chrono::milliseconds(options.feedback_period_ms() > 0
                                              ? options.feedback_period_ms() : 100);
    }

    /** @return Registry lookup; incomplete registrations are not dispatchable. */
    TaskPointer Get(RobotTaskType type) const {
        std::lock_guard<std::mutex> lock(mutex);
        const auto found = registry.find(static_cast<int>(type));
        return found == registry.end() || !found->second.ready ? nullptr : found->second.task;
    }

    /** @brief Install weak service bindings once, without touching TaskServer. */
    void InstallHooks(const TaskPointer& task) {
        const std::weak_ptr<State> weak = shared_from_this();
        const std::weak_ptr<TaskInterface> domain = task;
        Admission::Factory factory = [weak, domain] {
            const auto state = weak.lock();
            const auto owner = domain.lock();
            return state && owner ? state->Admit(owner) : nullptr;
        };
        if (auto nav = std::dynamic_pointer_cast<NavigationTask>(task)) {
            nav->SetMotionAdmission(std::move(factory));
        } else if (auto explore = std::dynamic_pointer_cast<ExplorationTask>(task)) {
            explore->SetMotionAdmission(std::move(factory));
            explore->SetGuardedSubmitNavigation([weak, domain](const auto& goal, const auto& permit) {
                const auto state = weak.lock();
                const auto parent = domain.lock();
                if (!state || !parent || !permit || !permit()) return false;
                const auto nav = std::dynamic_pointer_cast<NavigationTask>(
                    state->Get(RobotTaskType::ROBOT_TASK_NAVIGATION));
                if (!nav) return false;
                auto admission = state->Delegate(parent, nav);
                return admission && nav->SubmitGoalWithMotionPermit(goal, permit, std::move(admission));
            });
            explore->SetVersionedNavigationProbe(
                [weak](const auto& id, auto* result) {
                    const auto state = weak.lock();
                    const auto nav = state ? std::dynamic_pointer_cast<NavigationTask>(
                        state->Get(RobotTaskType::ROBOT_TASK_NAVIGATION)) : nullptr;
                    return nav && nav->GetResultForTask(id, result);
                },
                [weak](const auto& id) {
                    const auto state = weak.lock();
                    const auto nav = state ? std::dynamic_pointer_cast<NavigationTask>(
                        state->Get(RobotTaskType::ROBOT_TASK_NAVIGATION)) : nullptr;
                    if (!nav) throw std::runtime_error("delegated navigation backend unavailable");
                    const bool canceled = nav->CancelForTask(id);
                    // Scheduler preemption and the exploration dispatcher may
                    // cancel the same child concurrently. A duplicate Cancel
                    // can return while the first caller is still joining the
                    // worker. Observe its actual drain outside scheduler locks
                    // before treating that handoff as a permanent failure.
                    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
                    while (nav->GetSnapshot().task_id() == id && !nav->MotionDrained()) {
                        if (std::chrono::steady_clock::now() >= deadline)
                            throw std::runtime_error("delegated navigation cleanup unconfirmed");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                    return canceled;
                });
        }
    }

    /** @brief Configure a registered task outside the registry lock. */
    bool Prepare(const TaskPointer& task, RobotTaskType type, bool hooks,
                 const proto::TaskServerOptions& options, std::uint64_t epoch, bool initialize) {
        bool ok = false;
        bool installed = hooks;
        try {
            if (!installed) { InstallHooks(task); installed = true; }
            ok = !initialize || task->Initialize(options);
        } catch (...) { AERROR << "TaskScheduler task preparation failed type=" << static_cast<int>(type); }
        bool obsolete;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto found = registry.find(static_cast<int>(type));
            obsolete = shutting_down || configuration_epoch != epoch ||
                found == registry.end() || found->second.task != task;
            if (!obsolete) {
                found->second.hooks = installed;
                found->second.ready = ok && initialize;
            }
        }
        // A concurrently closed registration must not leave a new worker alive.
        if (obsolete && initialize) {
            try { task->Shutdown(); } catch (...) { ok = false; }
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            --preparing;
            wake.notify_all();
        }
        return ok && !obsolete;
    }

    /** @brief Register without replacing a live plugin with the same type. */
    void Register(const TaskPointer& task) {
        if (!task) return;
        const auto type = task->GetTaskType();
        proto::TaskServerOptions options;
        std::uint64_t epoch;
        bool initialize;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (shutting_down || preparing) return;
            const auto found = registry.find(static_cast<int>(type));
            if (found != registry.end()) {
                if (found->second.task != task) AWARN << "TaskScheduler duplicate task type rejected";
                return;
            }
            registry.emplace(static_cast<int>(type), Entry{task, type});
            options = server_options;
            epoch = configuration_epoch;
            initialize = open.load();
            ++preparing;
        }
        Prepare(task, type, false, options, epoch, initialize);
    }

    /** @brief Initialize registered plugins and open their new admission epoch. */
    bool Initialize(const proto::TaskServerOptions& options) {
        std::vector<Entry> plugins;
        std::uint64_t epoch;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (shutting_down || preparing || !roots.empty()) return false;
            if (open.load()) return true;
            server_options = options;
            if (options.has_scheduler()) ConfigureOptions(options.scheduler());
            epoch = ++configuration_epoch;
            for (auto& item : registry) {
                plugins.push_back(item.second);
                item.second.ready = false;
            }
            preparing += plugins.size();
        }
        bool ok = true;
        for (const auto& plugin : plugins)
            ok = Prepare(plugin.task, plugin.type, plugin.hooks, options, epoch, true) && ok;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (shutting_down || configuration_epoch != epoch) return false;
            open.store(ok);
        }
        return ok;
    }

    /** @return Native software cleanup, including work still awaiting admission. */
    static bool Drained(const TaskPointer& task) {
        if (!task) return true;
        if (const auto nav = std::dynamic_pointer_cast<NavigationTask>(task)) return nav->MotionDrained();
        if (const auto explore = std::dynamic_pointer_cast<ExplorationTask>(task)) return explore->MotionDrained();
        // Legacy plugins expose only their cancellation/lifecycle contract.
        // This fallback is a software state, not a physical stopping receipt.
        return !task->IsActive();
    }

    /** @brief Retire a generation and all still-retained delegated children. */
    static void Retire(const std::shared_ptr<Lease>& lease) {
        lease->retired.store(true);
        for (const auto& weak : lease->children)
            if (const auto child = weak.lock()) child->retired.store(true);
    }

    /** @return Root domain plus unique child domains, copied under mutex. */
    static std::vector<TaskPointer> Domains(const std::shared_ptr<Lease>& lease) {
        std::vector<TaskPointer> tasks;
        if (auto task = lease->task.lock()) tasks.push_back(std::move(task));
        for (const auto& weak : lease->children) {
            const auto child = weak.lock();
            const auto task = child ? child->task.lock() : nullptr;
            if (task && std::find(tasks.begin(), tasks.end(), task) == tasks.end()) tasks.push_back(task);
        }
        return tasks;
    }

    /** @return True after every synchronous parent/child admission has completed. */
    static bool PendingDrained(const std::shared_ptr<Lease>& lease) {
        if (lease->pending) return false;
        for (const auto& weak : lease->children)
            if (const auto child = weak.lock(); child && child->pending) return false;
        return true;
    }

    /** @return Conflict over the chassis or over the shared native navigation engine. */
    bool Conflicts(RobotTaskType left, RobotTaskType right) const {
        const auto engine = [](RobotTaskType type) {
            return type == RobotTaskType::ROBOT_TASK_NAVIGATION || type == RobotTaskType::ROBOT_TASK_EXPLORATION;
        };
        return (exclusive && IsNavigationClass(left) && IsNavigationClass(right)) || (engine(left) && engine(right));
    }

    /** @brief Create or reuse a root, releasing locks before canceling victims. */
    std::shared_ptr<Lease> Acquire(const TaskPointer& task, bool preliminary) {
        const auto type = task->GetTaskType();
        std::vector<std::shared_ptr<Lease>> victims;
        std::vector<TaskPointer> domains;
        std::uint64_t epoch;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto entry = registry.find(static_cast<int>(type));
            if (!open.load() || transition || entry == registry.end() ||
                !entry->second.ready || entry->second.task != task) return nullptr;
            const auto current = roots.find(task.get());
            if (current != roots.end() && !current->second->retired.load()) {
                auto lease = current->second;
                ++lease->revision;
                if (preliminary) lease->preliminary = true;
                else { lease->preliminary = false; ++lease->pending; }
                return lease;
            }
            for (const auto& item : roots) {
                if (item.first != task.get() && !Conflicts(item.second->type, type)) continue;
                if (item.first != task.get() && !item.second->retired.load() && !preemption) return nullptr;
                victims.push_back(item.second);
                const auto owned = Domains(item.second);
                for (const auto& domain : owned)
                    if (std::find(domains.begin(), domains.end(), domain) == domains.end()) domains.push_back(domain);
            }
            transition = true;
            epoch = configuration_epoch;
            for (const auto& victim : victims) Retire(victim);
        }
        bool clean = true;
        for (const auto& victim : domains) {
            try {
                const bool active = victim->IsActive();
                if (active || !Drained(victim)) {
                    const bool canceled = victim->Cancel();
                    if (!canceled && victim->IsActive()) clean = false;
                }
            } catch (...) { clean = false; }
        }
        // Exploration cancellation is queued to its owner. Wait for its real
        // runtime/dispatcher disposal, without blocking competing callbacks on
        // the scheduler mutex. Unresponsive cleanup leaves the retired barrier.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (clean && open.load()) {
            bool drained = true;
            try {
                for (const auto& domain : domains)
                    drained = Drained(domain) && !domain->IsActive() && drained;
            } catch (...) { clean = false; break; }
            std::unique_lock<std::mutex> wait_lock(mutex);
            for (const auto& victim : victims) drained = PendingDrained(victim) && drained;
            if (drained) break;
            if (std::chrono::steady_clock::now() >= deadline) { clean = false; break; }
            wake.wait_until(wait_lock, std::min(deadline, std::chrono::steady_clock::now() +
                                               std::chrono::milliseconds(5)));
        }
        std::lock_guard<std::mutex> lock(mutex);
        transition = false;
        wake.notify_all();
        const auto entry = registry.find(static_cast<int>(type));
        if (!open.load() || epoch != configuration_epoch || entry == registry.end() ||
            !entry->second.ready || entry->second.task != task) return nullptr;
        for (const auto& victim : victims) clean = PendingDrained(victim) && clean;
        if (!clean) return nullptr;
        for (const auto& victim : victims)
            if (auto owner = victim->task.lock()) roots.erase(owner.get());
        auto lease = std::make_shared<Lease>();
        lease->task = task;
        lease->type = type;
        lease->preliminary = preliminary;
        lease->pending = preliminary ? 0 : 1;
        roots[task.get()] = lease;
        return lease;
    }

    /** @brief Retain atomic permission and a non-reentrant completion callback. */
    std::shared_ptr<Admission> Handle(const std::shared_ptr<Lease>& lease,
                                      const std::shared_ptr<Lease>& parent = {}) {
        const std::weak_ptr<State> weak = shared_from_this();
        return std::make_shared<Admission>(
            [weak, lease, parent] {
                const auto state = weak.lock();
                return state && state->open.load() && !lease->retired.load() &&
                    (!parent || !parent->retired.load());
            },
            [weak, lease](bool) {
                if (const auto state = weak.lock()) {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (lease->pending) --lease->pending;
                    ++lease->revision;
                    state->wake.notify_all();
                }
            });
    }

    /** @return Ordinary admission, also used when TaskServer sees an active task. */
    std::shared_ptr<Admission> Admit(const TaskPointer& task) {
        const auto lease = Acquire(task, false);
        return lease ? Handle(lease) : nullptr;
    }

    /** @brief Attach a native child to the current exploration owner atomically. */
    std::shared_ptr<Admission> Delegate(const TaskPointer& task, const TaskPointer& nav) {
        std::shared_ptr<Lease> parent, child;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto root = roots.find(task.get());
            const auto registered = registry.find(static_cast<int>(RobotTaskType::ROBOT_TASK_NAVIGATION));
            if (!open.load() || transition || root == roots.end() || root->second->retired.load() ||
                root->second->type != RobotTaskType::ROBOT_TASK_EXPLORATION ||
                registered == registry.end() || !registered->second.ready || registered->second.task != nav ||
                roots.count(nav.get())) return nullptr;
            parent = root->second;
            child = std::make_shared<Lease>();
            child->task = nav;
            child->type = RobotTaskType::ROBOT_TASK_NAVIGATION;
            child->pending = 1;
            ++parent->revision;
            auto& children = parent->children;
            children.erase(std::remove_if(children.begin(), children.end(),
                [](const auto& old) { return old.expired(); }), children.end());
            children.push_back(child);
        }
        return Handle(child, parent);
    }

    /** @brief Remove only the exact idle generation checked outside the lock. */
    void Release(const TaskPointer& task, bool explicit_release) {
        std::shared_ptr<Lease> lease;
        std::vector<TaskPointer> domains;
        std::uint64_t revision;
        bool legacy_reservation = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto found = roots.find(task.get());
            if (transition || found == roots.end()) return;
            lease = found->second;
            if (explicit_release) { lease->preliminary = false; ++lease->revision; }
            if (lease->preliminary) {
                if (std::dynamic_pointer_cast<NavigationTask>(task) ||
                    std::dynamic_pointer_cast<ExplorationTask>(task)) return;
                // Legacy domains have no admission hook. An idle reservation
                // spans RequestActivation/SubmitGoal, but a terminal lifecycle
                // consumes it so finished legacy tasks can still be reclaimed.
                legacy_reservation = true;
            }
            if (!PendingDrained(lease)) return;
            revision = lease->revision;
            domains = Domains(lease);
        }
        try {
            if (legacy_reservation && !IsTerminalLifecycle(task->GetLifecycle())) return;
            for (const auto& domain : domains)
                if (domain->IsActive() || !Drained(domain)) return;
        } catch (...) { return; }
        std::lock_guard<std::mutex> lock(mutex);
        const auto found = roots.find(task.get());
        if (!transition && found != roots.end() && found->second == lease &&
            lease->revision == revision && (!lease->preliminary || legacy_reservation) && PendingDrained(lease)) {
            Retire(lease);
            roots.erase(found);
        }
    }

    /** @brief Reclaim domains without holding scheduler locks across task getters. */
    void Reap() {
        std::vector<TaskPointer> tasks;
        {
            std::lock_guard<std::mutex> lock(mutex);
            for (const auto& item : roots)
                if (auto task = item.second->task.lock()) tasks.push_back(std::move(task));
        }
        for (const auto& task : tasks) Release(task, false);
    }

    /** @brief Close admission before joining or shutting down any domain worker. */
    bool Close() {
        std::lock_guard<std::mutex> lock(mutex);
        if (shutting_down) return false;
        open.store(false);
        shutting_down = true;
        ++configuration_epoch;
        for (const auto& item : roots) Retire(item.second);
        wake.notify_all();
        return true;
    }

    /** @brief Dispose plugins; retain an unclean owner as a fail-closed barrier. */
    void Shutdown() {
        std::vector<TaskPointer> plugins;
        {
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait(lock, [this] { return preparing == 0; });
            for (const auto& item : registry) plugins.push_back(item.second.task);
        }
        bool clean = true;
        for (const auto& task : plugins) {
            try { task->Shutdown(); clean = Drained(task) && clean; }
            catch (...) { clean = false; }
        }
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& item : roots) clean = PendingDrained(item.second) && clean;
        if (clean) { registry.clear(); roots.clear(); }
        else {
            for (auto& item : registry) item.second.ready = false;
            AERROR << "TaskScheduler shutdown retains unconfirmed movement ownership";
        }
        shutting_down = false;
    }

    mutable std::mutex mutex;  ///< Registry and lease accounting; never held across task calls.
    std::condition_variable wake;  ///< Interruptible reaper and preparation completion.
    std::unordered_map<int, Entry> registry;  ///< One native plugin per automsgs task type.
    std::unordered_map<TaskInterface*, std::shared_ptr<Lease>> roots;  ///< Delegated navigation has no independent root.
    proto::TaskServerOptions server_options;  ///< Snapshot passed to domain initialization.
    std::atomic<bool> open{false};  ///< Shutdown invalidates every retained permit immediately.
    std::atomic<bool> running{false};  ///< Reaper enablement, separate from domain admission.
    std::atomic<std::uint64_t> run_epoch{0};  ///< A stopped worker cannot revive after Start.
    std::uint64_t configuration_epoch = 0;  ///< Invalidates preparation or takeover racing Shutdown.
    std::size_t preparing = 0;  ///< Registration/initialization outside the mutex.
    bool shutting_down = false;  ///< Rejects configuration changes during disposal.
    bool transition = false;  ///< Competing admission rejects while cancellation runs outside locks.
    bool exclusive = false;  ///< Optional chassis exclusivity for all navigation-class plugins.
    bool preemption = true;  ///< Policy governing takeover of another live root.
    std::chrono::milliseconds period{100};  ///< Interruptible session-reclaim period.
};

TaskScheduler::TaskScheduler(proto::SchedulerOptions options) : state_(std::make_shared<State>(options)) {}
TaskScheduler::~TaskScheduler() { try { Shutdown(); } catch (...) {} }
bool TaskScheduler::Initialize(const proto::TaskServerOptions& options) { return state_->Initialize(options); }
void TaskScheduler::RegisterTask(TaskPointer task) { state_->Register(task); }
TaskScheduler::TaskPointer TaskScheduler::GetTask(RobotTaskType type) const { return state_->Get(type); }
bool TaskScheduler::RequestActivation(TaskPointer task) { return task && static_cast<bool>(state_->Acquire(task, true)); }
void TaskScheduler::ReleaseActivation(TaskPointer task) { if (task) state_->Release(task, true); }
bool TaskScheduler::IsRunning() const { return state_->running.load(); }
bool TaskScheduler::ExclusiveNavigationEnabled() const {
    std::lock_guard<std::mutex> lock(state_->mutex); return state_->exclusive;
}
bool TaskScheduler::AllowPreemption() const {
    std::lock_guard<std::mutex> lock(state_->mutex); return state_->preemption;
}
void TaskScheduler::Shutdown() {
    if (!state_->Close()) return;
    Stop();
    state_->Shutdown();
}

bool TaskScheduler::Start() {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (!state_->open.load()) return false;
    if (state_->running.exchange(true)) return true;
    const auto epoch = ++state_->run_epoch;
    const auto state = state_;
    reclaim_thread_ = std::thread([state, epoch] {
        for (;;) {
            if (!state->running.load() || state->run_epoch.load() != epoch) return;
            state->Reap();
            std::unique_lock<std::mutex> lock(state->mutex);
            state->wake.wait_for(lock, state->period, [&] {
                return !state->running.load() || state->run_epoch.load() != epoch;
            });
        }
    });
    return true;
}

void TaskScheduler::Stop() {
    std::thread worker;
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        state_->running.store(false);
        ++state_->run_epoch;
        state_->wake.notify_all();
        worker = std::move(reclaim_thread_);
    }
    if (worker.joinable()) {
        if (worker.get_id() == std::this_thread::get_id()) worker.detach();
        else worker.join();
    }
}

std::vector<proto::ActiveTaskSnapshot> TaskScheduler::GetActiveSnapshots() const {
    std::vector<TaskPointer> tasks;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        for (const auto& item : state_->roots) {
            for (const auto& task : State::Domains(item.second))
                if (std::find(tasks.begin(), tasks.end(), task) == tasks.end()) tasks.push_back(task);
        }
    }
    std::vector<proto::ActiveTaskSnapshot> result;
    for (const auto& task : tasks) if (task->IsActive()) result.push_back(task->GetSnapshot());
    return result;
}

}  // namespace autonomy::task
