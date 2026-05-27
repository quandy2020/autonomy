/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <string>

namespace autonomy {
namespace tasks {

/** Minimal lifecycle surface used by system::Autonomy (via tasks::Task). */
class TaskInterface
{
public:
    TaskInterface() = default;
    virtual ~TaskInterface() = default;

    enum class TaskState {
        IDLE,
        RUNNING,
        PAUSED,
        COMPLETED,
        FAILED,
        CANCELED,
        STOPPED,
        SHUTDOWN
    };

    virtual TaskState GetState() const = 0;
    virtual bool Cancel() = 0;
    virtual void Shutdown() = 0;
};

}  // namespace tasks
}  // namespace autonomy
