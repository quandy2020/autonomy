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

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/localization/atla2/sensor/interface/sensor_base.hpp"
#include "autonomy/localization/atla2/sensor/lidar/deskew/lidar_deskew.hpp"
#include "autonomy/localization/atla2/sensor/lidar/driver/lidar_driver.hpp"
#include "autonomy/localization/atla2/sensor/lidar/filter/lidar_filter.hpp"
#include "autonomy/localization/atla2/sensor/lidar/preprocess/lidar_preprocess.hpp"

namespace autonomy::localization::atla2 {

//! Full lidar pipeline: driver → preprocess → deskew → filter → queue.
class LidarSensor : public SensorBase {
 public:
  explicit LidarSensor(std::string name = "lidar", LidarModel model = LidarModel::kGeneric);

  SensorKind Kind() const override { return SensorKind::kLidar; }
  const std::string& Name() const override { return name_; }

  bool Init(const Atla2Config& cfg) override;
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override { return running_; }
  TimeStamp LatestTime() const override;

  LidarDriverBase* Driver() { return driver_.get(); }
  LidarPreprocessor& Preprocess() { return preprocess_; }
  LidarDeskew& Deskew() { return deskew_; }
  LidarFilter& Filter() { return filter_; }

  void SetEnableDeskew(bool on) { enable_deskew_ = on; }
  void SetEnableFilter(bool on) { enable_filter_ = on; }

  //! Inject raw scan (runs pipeline).
  void FeedRaw(const LidarScan& raw, const std::vector<ImuSample>& imu = {},
               const Vec3& ba = Vec3::Zero(), const Vec3& bg = Vec3::Zero());

  bool Pop(LidarScan* out);

 private:
  void OnDriverScan(const LidarScan& raw);

  std::string name_;
  bool running_ = false;
  bool enable_deskew_ = true;
  bool enable_filter_ = true;
  std::unique_ptr<LidarDriverBase> driver_;
  LidarPreprocessor preprocess_;
  LidarDeskew deskew_;
  LidarFilter filter_;
  mutable std::mutex mu_;
  std::deque<LidarScan> q_;
  TimeStamp latest_ = kInvalidTime;
  std::vector<ImuSample> last_imu_;
  Vec3 ba_ = Vec3::Zero();
  Vec3 bg_ = Vec3::Zero();
};

}  // namespace autonomy::localization::atla2
