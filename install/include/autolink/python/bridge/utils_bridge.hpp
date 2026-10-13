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
#include <string>
#include <unordered_map>
#include <vector>

namespace autolink {
namespace python_support {

// Topology queries must live in libautolink; calling TopologyManager from
// _core.so duplicates singleton/static state and segfaults on Node create.

std::string GetChannelMsgType(const std::string& channel_name,
                              uint8_t sleep_s);
std::vector<std::string> GetChannels(uint8_t sleep_s);
std::unordered_map<std::string, std::vector<std::string>> GetChannelsInfo(
    uint8_t sleep_s);

std::vector<std::string> GetNodes(uint8_t sleep_s);
std::string GetNodeAttr(const std::string& node_name, uint8_t sleep_s);
std::vector<std::string> GetReadersOfNode(const std::string& node_name,
                                          uint8_t sleep_s);
std::vector<std::string> GetWritersOfNode(const std::string& node_name,
                                          uint8_t sleep_s);

std::vector<std::string> GetServices(uint8_t sleep_s);
std::string GetServiceAttr(const std::string& service_name, uint8_t sleep_s);

}  // namespace python_support
}  // namespace autolink
