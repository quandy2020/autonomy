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
#include <mutex>
#include <string>

#include "autonomy/localization/atla2/common/time.hpp"
#include "autonomy/localization/atla2/common/transform_tree.hpp"
#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

//! Soft / hard-trigger sync + extrinsic lookup via TransformTree.
class SensorSync {
 public:
  explicit SensorSync(TimeStamp tol_ns = SecToStamp(0.02));

  void SetTol(TimeStamp tol_ns) { tol_ns_ = tol_ns; }
  void SetExtrinsics(TransformTree* tree) { tree_ = tree; }
  TransformTree* Extrinsics() { return tree_; }

  void PushImu(const ImuSample& s);
  void PushImage(const ImageFrame& f);
  void PushLidar(const LidarScan& s);
  void PushGps(const GpsSample& s);
  void PushBaro(const BaroSample& s);
  void PushOpticalFlow(const OpticalFlowSample& s);

  //! Build a packet around newest image or lidar stamp.
  bool TryPop(SensorData* out);

  void Clear();

 private:
  TimeStamp tol_ns_;
  TransformTree* tree_ = nullptr;
  mutable std::mutex mu_;
  std::deque<ImuSample> imu_;
  std::deque<ImageFrame> images_;
  std::deque<LidarScan> lidars_;
  std::deque<GpsSample> gps_;
  std::deque<BaroSample> baro_;
  std::deque<OpticalFlowSample> flow_;
};

}  // namespace autonomy::localization::atla2
