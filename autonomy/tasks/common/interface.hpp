/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#ifndef AUTONOMY_TASKS_COMMON_INTERFACE_H_
#define AUTONOMY_TASKS_COMMON_INTERFACE_H_

namespace autonomy {
namespace tasks {

/**
 * @brief Minimal lifecycle surface for system::Autonomy and test doubles.
 */
class TaskInterface
{
public:
    TaskInterface() = default;
    virtual ~TaskInterface() = default;

    /** Lifecycle states for a navigation task session. */
    enum class TaskState {
        kIdle,
        kRunning,
        kCompleted,
        kFailed,
        kCanceled,
        kShutdown,
    };

    /** @return Current lifecycle state. */
    virtual TaskState GetState() const = 0;

    /**
     * @brief Cancels an in-progress navigation session.
     *
     * @return True if cancel was applied to an active session.
     */
    virtual bool Cancel() = 0;

    /** @brief Stops the task and releases active navigation. */
    virtual void Shutdown() = 0;
};

}  // namespace tasks
}  // namespace autonomy

#endif  // AUTONOMY_TASKS_COMMON_INTERFACE_H_
