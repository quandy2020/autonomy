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

#include <string>

#include "autoview/common/interaction/node_abstraction_iface.hpp"

namespace autoview {
namespace common {
namespace interaction {

class ClientAbstractionIface
{
public:
  virtual ~ClientAbstractionIface() = default;

  virtual NodeAbstractionIface::WeakPtr init(int argc, char ** argv, const std::string & name, bool anonymous_name) = 0;

  virtual bool ok() = 0;

  virtual void shutdown() = 0;
};

}  // namespace interaction
}  // namespace common
}  // namespace autoview