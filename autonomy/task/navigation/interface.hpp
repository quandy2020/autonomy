/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#ifndef AUTONOMY_TASK_NAVIGATION_INTERFACE_HPP_
#define AUTONOMY_TASK_NAVIGATION_INTERFACE_HPP_

namespace autonomy {
namespace task {
namespace navigation {

/**
 * @brief Minimal lifecycle surface for system::Autonomy and test doubles.
 */
class NavigatorInterface
{
public:
    NavigatorInterface() = default;
    virtual ~NavigatorInterface() = default;

    /** Lifecycle states for a navigation session. */
    enum class NavigatorState {
        kIdle,
        kRunning,
        kCompleted,
        kFailed,
        kCanceled,
        kShutdown,
    };

    /** @return Current lifecycle state. */
    virtual NavigatorState GetState() const = 0;

    /**
     * @brief Cancels an in-progress navigation session.
     *
     * @return True if cancel was applied to an active session.
     */
    virtual bool Cancel() = 0;

    /** @brief Stops the navigator and releases active navigation. */
    virtual void Shutdown() = 0;
};

}  // namespace navigation
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_NAVIGATION_INTERFACE_HPP_
