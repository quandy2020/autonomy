/*
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
#include <mutex>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace transform {
class TransformBroadcaster {
 public:
  /**
   * Define TransformBroadcaster::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(TransformBroadcaster)

  /**
   * @brief Constructor (needs a autolink::Node reference)
   * @param node The node to use for publishing
   */
  explicit TransformBroadcaster(const std::shared_ptr<::autolink::Node>& node);

  /**
   * @brief Constructor (needs a autolink::Node pointer)
   * @param node The node pointer to use for publishing (does not take
   * ownership)
   */
  explicit TransformBroadcaster(::autolink::Node* node);

  /**
   * @brief Send a TransformStamped message
   *     The stamped data structure includes frame_id, and time, and parent_id
   * already.
   * @param transform The transform to send
   */
  void SendTransform(const commsgs::geometry_msgs::TransformStamped& transform);

  /**
   * @brief Send a vector of TransformStamped messages
   *       The stamped data structure includes frame_id, and time, and
   * parent_id already.
   * @param transforms The transforms to send
   */
  void SendTransform(const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms);

 private:
  std::shared_ptr<::autolink::Node> node_;
  std::shared_ptr<::autolink::Writer<commsgs::geometry_msgs::TransformStampeds>> writer_;
};

}  // namespace transform
}  // namespace autonomy
