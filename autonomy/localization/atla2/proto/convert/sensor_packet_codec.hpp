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

#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

//! Field-mirror of proto SyncedSensorPacket (works without codegen).
//! When ATLA2_HAS_PROTO is defined, ToProto/FromProto map to the generated type.
struct SyncedSensorPacketDto {
  TimeStamp t = kInvalidTime;
  std::string frame_id = "body";

  std::vector<ImuSample> imu;
  ImageFrame image;
  ImageFrame image_right;
  LidarScan lidar;
  GpsSample gps;
  BaroSample baro;
  OpticalFlowSample optical_flow;

  bool has_image = false;
  bool has_image_right = false;
  bool has_lidar = false;
  bool has_imu = false;
  bool has_gps = false;
  bool has_baro = false;
  bool has_optical_flow = false;
};

bool SensorDataToDto(const SensorData& in, SyncedSensorPacketDto* out);
bool DtoToSensorData(const SyncedSensorPacketDto& in, SensorData* out);

//! Round-trip identity check (used by unit tests).
bool SensorDataRoundTripEqual(const SensorData& a, const SensorData& b);

}  // namespace autonomy::localization::atla2
