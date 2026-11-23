/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file driver.hpp
 * @brief ICL1122 / EVB1122 (UM10034P) UART mmWave radar driver.
 *
 * USB Type-C → ST Virtual COM (`/dev/ttyACM*`). Emits RadarFrame:
 * - `sensor_msgs/RadarScan` (primary)
 * - optional `RadarTracks` (demo targets)
 * - optional `PointCloud2` (Cartesian peaks / targets for autoviz)
 *
 * YAML: `backend: icl1122` (aliases `evb1122`, `um10034`, `iclegend`).
 * Params: `device`, `baud` (921600), `frame_id`, optional `register_config`,
 * `chirp_number`, `range_bins`, `range_resolution_m` or `bandwidth_hz`,
 * `center_frequency_hz`, `chirp_period_s`, `doppler_enable`, `max_abs_azimuth_deg`, `host_bringup`.
 */

#ifndef AUTODRIVER_RADAR_ICL1122_DRIVER_HPP_
#define AUTODRIVER_RADAR_ICL1122_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "autodriver/common/serial_byte_driver_base.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/radar/icl1122/protocol.hpp"
#include "autodriver/radar/icl1122/range_fft.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::Icl1122RadarDriver
 * @brief SerialByteDriverBase + demo / Range-FFT parsers → RadarFrame.
 */
class Icl1122RadarDriver : public SerialByteDriverBase<Icl1122RadarDriver> {
 public:
  AUTOLINK_SHARED_PTR_DEFINITIONS(Icl1122RadarDriver)
  DISALLOW_COPY_AND_ASSIGN(Icl1122RadarDriver)

  /**
   * @brief Store identity / params and construct parsers.
   * @param[in] id Sensor instance id (e.g. radar/front).
   * @param[in] params Expect `device`, optional baud / frame_id / bring-up.
   */
  Icl1122RadarDriver(SensorId id, DriverParams params);

  /** @brief Join the reader before destroying parsers. */
  ~Icl1122RadarDriver() override { Stop(); }

  /** @return SensorType::kRadar. */
  SensorType GetSensorType() const override { return SensorType::kRadar; }

  /**
   * @brief Optional JoySpace register bring-up after Connect.
   * @return false when `host_bringup` is set and configuration fails.
   */
  bool AfterConnect();

  /**
   * @brief CRTP: feed demo + MCU parsers; emit cloud on frames.
   * @param[in] data Bytes from Stream::Read.
   * @param[in] n Valid byte count.
   */
  void OnBytes(const std::uint8_t* data, std::size_t n);

  /** @brief Reset parsers after the worker joins. */
  void OnStopped();

 private:
  void EmitTargetCloud(const protocol::Icl1122TargetFrame& frame);
  void EmitRangeFftCloud(const protocol::Icl1122RangeFftFrame& frame);
  std::string ResolveRegisterConfigPath() const;

  protocol::Icl1122TargetParser target_parser_;
  protocol::Icl1122McuParser mcu_parser_;
  protocol::Icl1122RangeFftAssembler range_fft_;
  std::string frame_id_;
  bool host_bringup_{false};
  std::string register_config_;
  int cmd_response_ms_{200};
  double range_resolution_m_{0.375};
  double wavelength_m_{0.0125};          // c / centre frequency (UM10034P §2)
  double bandwidth_hz_{0.0};             // >0 overrides range_resolution_m_
  double chirp_period_s_{0.0};           // >0 skips the frame-period estimate
  double measured_frame_period_s_{0.0};  // EMA of consecutive Range-FFT frames
  std::uint64_t last_range_fft_stamp_ns_{0};
  bool doppler_enable_{true};   // UM10034P §5.3 2DFFT velocity from chirp phase
  double doppler_sign_{1.0};    // -1 flips the IQ phase convention
  double max_abs_azimuth_rad_{0.7505};  // UM10034P §2.3: H plane ±43°
  float peak_ratio_{4.f};  // CFAR multiplier vs far-bin noise
  std::size_t skip_near_bins_{3};
  std::size_t max_peaks_{1};     // 1 = single person; 0 = all CFAR peaks
  float min_peak_frac_{0.45f};   // drop peaks < max_amp * frac
  bool publish_spectrum_{false};
  double rx_spacing_m_{0.00625};  // RX baseline ≈ λ/2 @ 24 GHz
  // Range-rate from primary (strongest) peak — no Doppler FFT without GUI.
  bool have_prev_nearest_{false};
  float prev_nearest_m_{0.f};
  std::uint64_t prev_stamp_ns_{0};
};

/**
 * @brief Factory for RadarBackendRegistry (`icl1122` / aliases).
 * @param[in] id Sensor id from YAML.
 * @param[in] params DriverParams map.
 * @return Owning Icl1122RadarDriver*.
 */
SensorDriver* CreateIcl1122RadarDriver(const SensorId& id,
                                       const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_ICL1122_DRIVER_HPP_
