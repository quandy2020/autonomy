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

#pragma once

#include "autonomy/tools/aviz/common/tool.hpp"

namespace aviz {
namespace common {

/**
 * @brief ToolManager
 * Manages interactive tools
 */
class ToolManager
{
public:
    ToolManager() = default;
    ~ToolManager() = default;

    void addTool(Tool* tool);
    void removeTool(Tool* tool);
    Tool* getCurrentTool() const;
    void setCurrentTool(Tool* tool);

private:
    Tool* current_tool_;
};

}  // namespace common
}  // namespace aviz
