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

#include <string>
#include <vector>
#include <tuple>
#include <functional>
#include <type_traits>
#include <memory>
#include <atomic>
#include <any>
#include <utility>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tasks {
namespace common {

class TaskInterface 
{
public:
   /**
    * Define TaskInterface::SharedPtr type
    */
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskInterface)

    /**
     * @brief A constructor for autonomy::tasks::TaskInterface
     * @param options Additional options to control creation of the node.
     */
    TaskInterface() = default;
    
    /**
     * @brief A Destructor for autonomy::tasks::TaskInterface
     */
    virtual ~TaskInterface() = default;

    /**
     * @brief Enumeration representing the possible states of a task
     * 
     * This enum class defines the various states that a task can be in during its lifecycle:
     * - IDLE: Task has been created but not yet started
     * - RUNNING: Task is currently executing
     * - PAUSED: Task execution has been temporarily suspended
     * - COMPLETED: Task has finished executing successfully
     * - FAILED: Task has terminated due to an error or exception
     * - CANCELED: Task was explicitly canceled before completion
     * - STOPPED: Task was explicitly stopped before completion
     */
    enum class TaskState {
        IDLE,       ///< Task is created but not started
        RUNNING,    ///< Task is actively executing
        PAUSED,     ///< Task execution is temporarily suspended
        COMPLETED,  ///< Task finished successfully
        FAILED,     ///< Task terminated due to error
        CANCELED,   ///< Task was explicitly canceled
        STOPPED     ///< Task was explicitly stopped
    };

    /**
     * @brief Starts the task with the provided arguments.
     * 
     * This function template captures any number of arguments, stores them in a vector of type-erased values (std::any),
     * and forwards them to the implementation-specific StartImpl method.
     * 
     * @tparam Args Variadic template parameter pack representing the types of arguments to be passed
     * @param args Arguments to be forwarded to the StartImpl method
     * @return bool True if the task was successfully started, false otherwise
     */
    template<typename... Args>
    bool Start(Args&&... args) {
        std::vector<std::any> captured_args;
        (captured_args.emplace_back(std::forward<Args>(args)), ...);
        return StartImpl(std::move(captured_args));
    }
    
    /**
     * @brief Resumes a paused task
     * @return bool True if the task was successfully resumed, false otherwise
     */
    virtual bool Resume() = 0;

    /**
     * @brief Cancels the task execution
     * @return bool True if the task was successfully canceled, false otherwise
     */
    virtual bool Cancel() = 0;

    /**
     * @brief Stops the task execution
     * @return bool True if the task was successfully stopped, false otherwise
     */
    virtual bool Stop() = 0;

    /**
     * @brief Gets the current state of the task
     * @return TaskState The current state of the task
     */
    virtual TaskState GetState() const = 0;

    /**
     * @brief Executes the task's callback function
     * 
     * This function is typically called periodically or when specific events occur
     * to execute the main logic of the task.
     */
    virtual void ExecuteCallback() = 0;

    /**
     * @brief Gets the name of the task
     * @return std::string The name identifier of the task
     */
    virtual std::string GetName() const = 0;

protected:
    virtual bool StartImpl(std::vector<std::any>&& args) = 0;
};

}  // namespace common
}  // namespace tasks
}  // namespace autonomy