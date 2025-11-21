/**
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

#include <memory>
#include <string>
#include <vector>

#include "autolink/proto/dag_conf.pb.h"

#include "autolink/class_loader/class_loader_manager.hpp"
#include "autolink/component/component.hpp"
#include "autolink/mainboard/module_argument.hpp"

namespace autolink {
namespace mainboard {

using autolink::proto::DagConfig;

class ModuleController
{
public:
    explicit ModuleController(const ModuleArgument& args);
    virtual ~ModuleController() = default;

    bool Init();
    bool LoadAll();
    void Clear();

private:
    bool LoadModule(const std::string& path);
    bool LoadModule(const DagConfig& dag_config);
    int GetComponentNum(const std::string& path);
    int total_component_nums = 0;
    bool has_timer_component = false;

    ModuleArgument args_;
    class_loader::ClassLoaderManager class_loader_manager_;
    std::vector<std::shared_ptr<ComponentBase>> component_list_;
};

inline ModuleController::ModuleController(const ModuleArgument& args)
    : args_(args) {}

inline bool ModuleController::Init() {
    return LoadAll();
}

}  // namespace mainboard
}  // namespace autolink