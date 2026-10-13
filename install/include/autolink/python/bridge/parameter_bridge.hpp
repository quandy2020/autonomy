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

#include <memory>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/parameter/parameter.hpp"

namespace autolink {
namespace python_support {

// Opaque wrappers so pybind11 TUs never include Client<>/Service<> headers
// (implicit ParameterClient/Server dtors would instantiate templates in _core).

class ParameterServerHandle {
public:
    explicit ParameterServerHandle(const std::shared_ptr<Node>& node);
    ~ParameterServerHandle();

    ParameterServerHandle(const ParameterServerHandle&) = delete;
    ParameterServerHandle& operator=(const ParameterServerHandle&) = delete;

    void SetParameter(const Parameter& parameter);
    bool GetParameter(const std::string& name, Parameter* parameter);
    void ListParameters(std::vector<Parameter>* parameters);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class ParameterClientHandle {
public:
    ParameterClientHandle(const std::shared_ptr<Node>& node,
                          const std::string& server_node_name);
    ~ParameterClientHandle();

    ParameterClientHandle(const ParameterClientHandle&) = delete;
    ParameterClientHandle& operator=(const ParameterClientHandle&) = delete;

    bool SetParameter(const Parameter& parameter);
    bool GetParameter(const std::string& name, Parameter* parameter);
    bool ListParameters(std::vector<Parameter>* parameters);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace python_support
}  // namespace autolink
