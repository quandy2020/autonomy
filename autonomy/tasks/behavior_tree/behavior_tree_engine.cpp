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

#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

BehaviorTreeEngine::BehaviorTreeEngine(
  const std::vector<std::string>& plugin_libraries, 
  common::TaskInterface::SharedPtr task)
{
    BT::SharedLibrary loader;
    for (const auto & p : plugin_libraries) {
        factory_.registerFromPlugin(loader.getOSName(p));
    }
}

BtStatus BehaviorTreeEngine::Run(
  BT::Tree* tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    // Loop until something happens with task completes
    try 
    {
        while (result == BT::NodeStatus::RUNNING) 
        {
            if (cancelRequested()) {
                tree->haltTree();
                return BtStatus::CANCELED;
            }

            result = tree->tickOnce();

            onLoop();

            std::this_thread::sleep_for(std::chrono::milliseconds(loopTimeout));
        }
    } catch (const std::exception & ex) {
        LOG(ERROR) << absl::StrCat("Behavior tree threw exception: ", 
            ex.what(), " . Exiting with failure.");
        return BtStatus::FAILED;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::CreateTreeFromText(const std::string & xml_string, BT::Blackboard::Ptr blackboard)
{
    return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree BehaviorTreeEngine::CreateTreeFromFile(const std::string & file_path, BT::Blackboard::Ptr blackboard)
{
    return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void BehaviorTreeEngine::HaltAllActions(BT::Tree & tree)
{
    // this halt signal should propagate through the entire tree.
    tree.haltTree();
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy