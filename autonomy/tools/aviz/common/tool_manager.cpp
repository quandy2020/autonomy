/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/common/tool_manager.hpp"

namespace aviz {
namespace common {

void ToolManager::addTool(Tool* tool) {
  (void)tool;
  // TODO: Implement
}

void ToolManager::removeTool(Tool* tool) {
  (void)tool;
  // TODO: Implement
}

Tool* ToolManager::getCurrentTool() const { return current_tool_; }

void ToolManager::setCurrentTool(Tool* tool) { current_tool_ = tool; }

}  // namespace common
}  // namespace aviz
