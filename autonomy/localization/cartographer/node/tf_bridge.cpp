/*
 * Copyright 2016 The Cartographer Authors
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

#include "autonomy/localization/cartographer/node/tf_bridge.hpp"

#include <algorithm>

#include <automsgs/msgs/time_utils.hpp>
#include <glog/logging.h>

#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/time_conversion.hpp"
#include "autonomy/transform/tf2/exceptions.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

TfBridge::TfBridge(const std::string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const transform::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string& frame_id) const {
    float timeout = static_cast<float>(lookup_transform_timeout_sec_);
    try {
        const automsgs::msgs::builtin_interfaces::Time latest_tf_time =
            buffer_
                ->lookupTransform(tracking_frame_, frame_id,
                                  automsgs::msgs::builtin_interfaces::Time(),
                                  timeout)
                .header().stamp();
        const automsgs::msgs::builtin_interfaces::Time requested_time = ToCommsgs(time);

        if (automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
                latest_tf_time) >=
            automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
                requested_time)) {
            // Keep a short timeout so Buffer can fall back on future
            // extrapolation instead of immediately returning empty ":timeout".
            timeout = std::min(timeout, 0.05f);
        }
        return std::make_unique<::cartographer::transform::Rigid3d>(
            ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
                                               requested_time, timeout)));
    } catch (const autonomy::transform::tf2::TransformException& ex) {
        LOG_EVERY_N(WARNING, 50) << "TF " << frame_id << " -> "
                                 << tracking_frame_ << ": " << ex.what();
    }
    return nullptr;
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
