/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/exploration/common/explorer_interface.hpp"

namespace autonomy {
namespace exploration {
namespace common {

ExplorerInterface::ExplorerInterface(const proto::ExplorationOptions& options,
                                     std::string name)
    : options_(options), name_(std::move(name))
{
}

void ExplorerInterface::Configure(const proto::ExplorationOptions& options,
                                  const std::string& name)
{
    options_ = options;
    name_ = name;
}

}  // namespace common
}  // namespace exploration
}  // namespace autonomy
