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

#include "autonomy/localization/atla2/sensor/interface/sensor_base.hpp"

namespace autonomy::localization::atla2 {

struct CameraIntrinsics {
  double fx = 320.0;
  double fy = 320.0;
  double cx = 320.0;
  double cy = 240.0;
  double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;
  int width = 640;
  int height = 480;
};

//! Camera adapter: ingest frames, optional undistort hook, ring buffer.
class CameraSensor : public SensorBase {
 public:
  explicit CameraSensor(std::string name = "camera");

  SensorKind Kind() const override { return SensorKind::kCamera; }
  const std::string& Name() const override { return name_; }

  bool Init(const Atla2Config& cfg) override;
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override { return running_; }
  TimeStamp LatestTime() const override;

  void SetIntrinsics(const CameraIntrinsics& K) { K_ = K; }
  const CameraIntrinsics& Intrinsics() const { return K_; }

  void Push(const ImageFrame& frame);
  bool Pop(ImageFrame* out);
  //! Undistort placeholder (copies if no distortion).
  static ImageFrame Undistort(const ImageFrame& in, const CameraIntrinsics& K);

 private:
  std::string name_;
  CameraIntrinsics K_;
  bool running_ = false;
  mutable std::mutex mu_;
  std::deque<ImageFrame> q_;
  TimeStamp latest_ = kInvalidTime;
};

}  // namespace autonomy::localization::atla2
