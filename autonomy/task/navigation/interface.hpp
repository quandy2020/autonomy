/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file interface.hpp
 * @brief Minimal lifecycle surface for navigators and test doubles.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_INTERFACE_HPP_
#define AUTONOMY_TASK_NAVIGATION_INTERFACE_HPP_

namespace autonomy {
namespace task {
namespace navigation {

/**
 * @brief Minimal lifecycle surface for navigators and test doubles.
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

    /**
     * @brief Stops the navigator and releases active navigation.
     */
    virtual void Shutdown() = 0;
};

}  // namespace navigation
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_NAVIGATION_INTERFACE_HPP_
