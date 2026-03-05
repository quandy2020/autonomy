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

#ifndef AVIZ_COMMON__INTERACTION__HANDLER_MANAGER_LISTENER_HPP_
#define AVIZ_COMMON__INTERACTION__HANDLER_MANAGER_LISTENER_HPP_

#include "autonomy/tools/aviz/common/interaction/forwards.hpp"

namespace aviz {
namespace common {
namespace interaction {

class HandlerManagerListener {
 public:
  virtual ~HandlerManagerListener() = default;

  virtual void onHandlerRemoved(CollObjectHandle handle) = 0;
};

}  // namespace interaction
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__INTERACTION__HANDLER_MANAGER_LISTENER_HPP_
