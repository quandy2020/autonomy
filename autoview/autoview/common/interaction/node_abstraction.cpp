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


 #include "autoview/common/interaction/node_abstraction.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"

namespace autoview {
namespace common {
namespace interaction {

NodeAbstraction::NodeAbstraction(const std::string & node_name)
: raw_node_(autolink::CreateNode(node_name))
{}

std::string NodeAbstraction::get_node_name() const
{
  return raw_node_->Name();
}

std::map<std::string, std::vector<std::string>> NodeAbstraction::get_topic_names_and_types() const
{
  // autolink Node 暂无对应 API，先返回空；后续可从 topology 等获取
  return {};
}

}  // namespace interaction
}  // namespace common
}  // namespace autoview
