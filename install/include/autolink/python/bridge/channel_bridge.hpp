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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "autolink/node/node.hpp"

namespace autolink {
namespace python_support {

// Non-template bridge so pybind11 TUs never instantiate Writer/Reader templates
// (duplicate static state across libautolink.so and _core.so caused Init segfaults).

class ChannelWriter {
public:
    ChannelWriter(const std::shared_ptr<Node>& node, const std::string& channel,
                  const std::string& data_type, uint32_t qos_depth);
    ~ChannelWriter();
    bool Write(const std::string& bytes);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class ChannelReader {
public:
    using Callback = std::function<void(const std::string& bytes)>;

    ChannelReader(const std::shared_ptr<Node>& node, const std::string& channel,
                  Callback callback, const std::string& data_type);
    ~ChannelReader();

    ChannelReader(const ChannelReader&) = delete;
    ChannelReader& operator=(const ChannelReader&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

std::shared_ptr<Node> CreatePyNode(const std::string& name);
bool RegisterPythonMessageDescriptor(const std::string& serialized_desc);

class ServiceServer {
public:
    using Callback =
        std::function<std::string(const std::string& request_bytes)>;

    ServiceServer(const std::shared_ptr<Node>& node,
                  const std::string& service_name, Callback callback,
                  const std::string& data_type = "RawData");
    ~ServiceServer();

    ServiceServer(const ServiceServer&) = delete;
    ServiceServer& operator=(const ServiceServer&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class ServiceClient {
public:
    ServiceClient(const std::shared_ptr<Node>& node,
                  const std::string& service_name,
                  const std::string& data_type = "RawData");
    ~ServiceClient();

    std::string SendRequest(const std::string& request_bytes,
                            int timeout_sec = 5);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace python_support
}  // namespace autolink
