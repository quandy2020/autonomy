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
#include <string>
#include <thread>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/time/time.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/localization/amcl/amcl_localization.hpp"
#include "autonomy/localization/common/localization_interface.hpp"
#include "autonomy/localization/proto/localization_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/transform_broadcaster.hpp"

namespace autonomy {
namespace localization {

class Cartographer : public LocalizationInterface {
 public:
  explicit Cartographer(const proto::LocalizationOptions& options, const std::string& node_name = "cartographer");
  ~Cartographer() override;

  bool Start();
  bool Stop();
  bool SetInitialPose(const commsgs::geometry_msgs::PoseWithCovariance& pose);
  bool GetPose(commsgs::geometry_msgs::PoseWithCovariance& pose);

 private:
  void HandleLaserScan(const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& laser_scan);
};

}  // namespace localization
}  // namespace autonomy