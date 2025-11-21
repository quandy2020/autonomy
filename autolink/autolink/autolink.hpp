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
#include <utility>

// Fix FastCDR TEMPLATE_SPEC issue - must be included before any FastCDR headers
#include <fastcdr/config.h>

#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/component/component.hpp"
#include "autolink/node/node.hpp"
#include "autolink/task/task.hpp"
#include "autolink/time/time.hpp"
#include "autolink/timer/timer.hpp"

namespace autolink {

std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space = "");

}  // namespace autolink