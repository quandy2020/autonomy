/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/common/signal_processing/lowpass_filter.hpp"

namespace autonomy {
namespace common {
namespace signal_processing {

// geometry_msgs::Twist LowpassFilterTwist::filter(const geometry_msgs::Twist & u)
// {
//   if (x_) {
//     x_->mutable_linear()->set_x(gain_ * x_->linear().x() + (1.0 - gain_) * u.linear().x());
//     x_->mutable_linear()->set_y(gain_ * x_->linear().y() + (1.0 - gain_) * u.linear().y());
//     x_->mutable_linear()->set_z(gain_ * x_->linear().z() + (1.0 - gain_) * u.linear().z());

//     x_->mutable_angular()->set_x(gain_ * x_->angular().x() + (1.0 - gain_) * u.angular().x());
//     x_->mutable_angular()->set_y(gain_ * x_->angular().y() + (1.0 - gain_) * u.angular().y());
//     x_->mutable_angular()->set_z(gain_ * x_->angular().z() + (1.0 - gain_) * u.angular().z());

//     return x_.get();
//   }

//   x_ = u;
//   return x_.get();
// }

}  // namespace signal_processing
}  // namespace common
}  // namespace autonomy