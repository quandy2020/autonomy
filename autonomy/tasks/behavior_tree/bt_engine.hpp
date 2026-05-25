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

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

/** @brief Loads BT plugins and runs tick loops on a tree. */
class BtEngine
{
public:
    explicit BtEngine(const std::vector<std::string>& plugin_libraries,
                      const std::string& plugin_lib_path = "");

    virtual ~BtEngine() {}

    BtStatus Run(
        BT::Tree* tree, std::function<void()> onLoop,
        std::function<bool()> cancelRequested,
        std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

    BT::Tree CreateTreeFromText(const std::string& xml_string,
                                BT::Blackboard::Ptr blackboard);

    BT::Tree CreateTreeFromFile(const std::string& file_path,
                                BT::Blackboard::Ptr blackboard);

    void HaltAllActions(BT::Tree& tree);

protected:
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Groot2Publisher> groot_monitor_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
