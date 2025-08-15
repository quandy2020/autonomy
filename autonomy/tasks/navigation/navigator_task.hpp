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

#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"

namespace autonomy {
namespace tasks {
namespace navigation {

class NavigatorTask
{
public:
    NavigatorTask();
    ~NavigatorTask();

private:
    // behavior_tree engine
    // behavior_tree::BehaviorTreeEngine::SharedPtr bt_engine_{nullptr};
};

}  // namespace navigation
}  // namespace tasks
}  // namespace autonomy