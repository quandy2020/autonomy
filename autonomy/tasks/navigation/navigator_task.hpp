/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once 

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <future>

#include "autonomy/tasks/proto/navigation.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/thread_pool.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/common/task_interface.hpp"

namespace autonomy {
namespace tasks {
namespace navigation {

class NavigatorTask : public common::TaskInterface
{
public:
    using TaskState = common::TaskInterface::TaskState;

    /**
    * Define NavigatorTask::SharedPtr type
    */
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigatorTask)

    /**
     * @brief A constructor for autonomy::tasks::NavigatorTask
     * @param options Additional options to control creation of the node.
     */
    NavigatorTask();
    
    /**
     * @brief A constructor for autonomy::tasks::NavigatorTask
     * @param options Additional options to control creation of the node.
     */
    NavigatorTask(const std::string& name);
    
    /**
     * @brief A Destructor for autonomy::tasks::TaskInterface
     */
    ~NavigatorTask();
   
    /**
     * @brief Resumes a paused task
     * @return bool True if the task was successfully resumed, false otherwise
     */
    bool Resume() override;
   
    /**
     * @brief Cancels the task execution
     * @return bool True if the task was successfully canceled, false otherwise
     */
    bool Cancel() override;

    /**
     * @brief Stops the task execution
     * @return bool True if the task was successfully stopped, false otherwise
     */
    bool Stop() override;

    /**
     * @brief Shuts down the task
     * 
     * This function is responsible for performing any necessary cleanup or resource release
     * when the task is being shut down. It is typically called when the task is being
     * stopped or when the program is exiting.
     */
    void Shutdown() override;

    /**
     * @brief Gets the current state of the task
     * @return TaskState The current state of the task
     */
    TaskState GetState() const override;

    /**
     * @brief Gets the name of the task
     * @return std::string The name identifier of the task
     */
    std::string GetName() const override;

private:

    bool StartImpl(std::vector<std::any>&& args) override;

    // common::ThreadPool pool_;
    std::string name_;
    std::atomic<TaskState> state_;

    /// @brief options for navigation
    proto::NavigationOptions options_;
};

}  // namespace navigation
}  // namespace tasks
}  // namespace autonomy