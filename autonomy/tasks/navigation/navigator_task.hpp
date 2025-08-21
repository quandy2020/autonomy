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

#include "autonomy/tasks/proto/navigate_to_pose.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/thread_pool.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/common/task_interface.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"

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
     * @brief Gets the current state of the task
     * @return TaskState The current state of the task
     */
    TaskState GetState() const override;

    /**
     * @brief Executes the task's callback function
     * 
     * This function is typically called periodically or when specific events occur
     * to execute the main logic of the task.
     */
    void ExecuteCallback() override;

    /**
     * @brief Gets the name of the task
     * @return std::string The name identifier of the task
     */
    std::string GetName() const override;

private:

    bool StartImpl(std::vector<std::any>&& args) override;

    /**
     * @brief handle goal for planning A --> B
     */
    bool HandleGoalReceived(const proto::NavigateToPose::Goal& goal);

    /**
     * @brief Replace current BT with another one
     * @param bt_xml_filename The file containing the new BT, uses default filename if empty
     * @return bool true if the resulting BT correspond to the one in bt_xml_filename. false
     * if something went wrong, and previous BT is maintained
     */
    bool LoadBehaviorTree(const std::string& bt_xml_filename = "");

    /**
     * @brief Getter function for BT Blackboard
     * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
     */
    BT::Blackboard::Ptr GetBlackboard() const
    {
        return blackboard_;
    }

    /**
     * @brief Getter function for default BT XML filename
     * @return string Containing default BT XML filename
     */
    std::string GetDefaultBTFilename() const
    {
        return default_bt_xml_filename_;
    }

    /**
     * @brief Getter function for current BT XML filename
     * @return string Containing current BT XML filename
     */
    std::string GetCurrentBTFilename() const
    {
        return current_bt_xml_filename_;
    }

    /**
     * @brief Get navigator's default BT
     * @return string Filepath to default XML
     */
    std::string GetDefaultBTFilepath();

    /**
     * @brief Getter function for the current BT tree
     * @return BT::Tree Current behavior tree
     */
    const BT::Tree& GetTree() const
    {
        return tree_;
    }

    /**
     * @brief Function to halt the current tree. It will interrupt the execution of RUNNING nodes
     * by calling their halt() implementation (only for Async nodes that may return RUNNING)
     * This should already done for all the exit states of the action but preemption
     */
    void HaltTree()
    {
        tree_.haltTree();
    }

    /**
     * @brief A callback that defines execution that happens on one iteration through the BT
     * Can be used to publish action feedback
     */
    void OnLoop();

    /**
     * @brief Goal pose initialization on the blackboard
     * @param goal goal message to process
     * @return bool if goal was initialized successfully to be processed
     */
    bool InitializeGoalPose(const commsgs::geometry_msgs::PoseStamped& goal);

    /**
     * @brief Attempts to extract and return a value of type T from a std::any object
     * Throws std::runtime_error if the contained type doesn't match T
     */
    template<typename T>
    T GetArgument(const std::any& arg, std::size_t index, const std::string& arg_name) 
    {
        try {
            return std::any_cast<T>(arg);
        } catch (const std::bad_any_cast&) {
            throw std::runtime_error("Argument " + std::to_string(index) + 
                " (" + arg_name + ") has wrong type");
        }
    }

    /**
     * @brief Validates that the number of provided arguments matches the expected count
     * Throws std::runtime_error if the argument count doesn't match expectations
     */
    void CheckArgumentCount(const std::vector<std::any>& args, 
        std::size_t expected, const std::string& task_name) 
    {
        if (args.size() != expected) {
            throw std::runtime_error("Task " + task_name + " expects " + 
                std::to_string(expected) + " arguments, got " + std::to_string(args.size()));
        }
    }

    // Behavior Tree to be executed when goal is received
    BT::Tree tree_;

    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;
    std::string goal_blackboard_id_;

    // The XML file that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    // behavior_tree engine
    behavior_tree::BehaviorTreeEngine::SharedPtr bt_{nullptr};

    // Libraries to pull plugins (BT Nodes) from
    std::vector<std::string> plugin_lib_names_;

    // Error code id names
    std::vector<std::string> error_code_names_;

    // Duration for each iteration of BT execution
    std::chrono::milliseconds bt_loop_duration_;

    // Default timeout value while waiting for response from a server
    std::chrono::milliseconds default_server_timeout_;

    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;

    // should the BT be reloaded even if the same xml filename is requested?
    bool always_reload_bt_xml_ = false;

    // Main task thread handler
    std::future<void> execution_future_;

    // common::ThreadPool pool_;
    std::string name_;
    std::atomic<TaskState> state_;
};

}  // namespace navigation
}  // namespace tasks
}  // namespace autonomy