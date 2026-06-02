/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#ifndef AUTONOMY_NAVIGATOR_COMMON_INTERFACE_H_
#define AUTONOMY_NAVIGATOR_COMMON_INTERFACE_H_

namespace autonomy {
namespace navigator {

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

}  // namespace navigator
}  // namespace autonomy

#endif  // AUTONOMY_NAVIGATOR_COMMON_INTERFACE_H_
