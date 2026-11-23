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
 * @file driver.cpp
 * @brief ICL1122 / EVB1122 UART mmWave radar driver (implementation).
 */

#include "autodriver/radar/icl1122/driver.hpp"

#include <cmath>
#include <cstdint>
#include <vector>
#include <filesystem>
#include <utility>

#include "autodriver/conf/conf.hpp"
#include "autodriver/lidar/point_cloud2_layout.hpp"
#include "autodriver/radar/backend_register.hpp"
#include "autodriver/radar/icl1122/host_cfg.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {
namespace {

constexpr double kLightSpeed = 299792458.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

std::filesystem::path PackageRoot() {
  return std::filesystem::path(conf::kDefaultConfigDir).parent_path();
}

void SetTrackUuid(automsgs::msgs::unique_identifier_msgs::UUID* uuid,
                  std::uint32_t index) {
  std::string bytes(16, '\0');
  bytes[0] = static_cast<char>(index & 0xFF);
  bytes[1] = static_cast<char>((index >> 8) & 0xFF);
  bytes[2] = static_cast<char>((index >> 16) & 0xFF);
  bytes[3] = static_cast<char>((index >> 24) & 0xFF);
  uuid->set_uuid(bytes);
}

template <typename RepeatedFloat>
void FillZeroCov6(RepeatedFloat* cov) {
  cov->Clear();
  for (int i = 0; i < 6; ++i) {
    cov->Add(0.f);
  }
}

}  // namespace

Icl1122RadarDriver::Icl1122RadarDriver(SensorId id, DriverParams params)
    : SerialByteDriverBase<Icl1122RadarDriver>(std::move(id), std::move(params),
                                               /*read_timeout_ms=*/50),
      frame_id_(GetString(params_, "frame_id", "radar_link")),
      register_config_(GetString(params_, "register_config", "")),
      cmd_response_ms_(ParseInt(params_, "cmd_response_ms", 200)),
      range_resolution_m_(
          ParseDouble(params_, "range_resolution_m", 0.375)),
      peak_ratio_(static_cast<float>(
          ParseDouble(params_, "peak_ratio", 4.0))),
      skip_near_bins_(static_cast<std::size_t>(
          ParseInt(params_, "skip_near_bins", 3))),
      max_peaks_(static_cast<std::size_t>(ParseInt(params_, "max_peaks", 1))),
      min_peak_frac_(static_cast<float>(
          ParseDouble(params_, "min_peak_frac", 0.45))),
      publish_spectrum_(ParseBool(params_, "publish_spectrum", false)),
      rx_spacing_m_(ParseDouble(params_, "rx_spacing_m", 0.00625)),
      bandwidth_hz_(ParseDouble(params_, "bandwidth_hz", 0.0)),
      chirp_period_s_(ParseDouble(params_, "chirp_period_s", 0.0)),
      doppler_enable_(ParseBool(params_, "doppler_enable", true)),
      doppler_sign_(ParseDouble(params_, "doppler_sign", 1.0)),
      max_abs_azimuth_rad_(
          ParseDouble(params_, "max_abs_azimuth_deg", 43.0) * kDegToRad) {
  // Default false: UM10034 §4.5 Demo AA FF targets. JoySpace §5 FFT needs
  // an explicit host_bringup: true (see config/radar/icl1122.yaml).
  host_bringup_ = ParseBool(params_, "host_bringup", false);
  const auto chirps =
      static_cast<std::uint32_t>(ParseInt(params_, "chirp_number", 16));
  const auto bins =
      static_cast<std::uint32_t>(ParseInt(params_, "range_bins", 8));
  const auto rx =
      static_cast<std::uint32_t>(ParseInt(params_, "num_rx", 2));
  range_fft_.SetConfig(chirps, bins, rx);

  // UM10034P §2: 24 GHz ISM, 24-25 GHz sweep. §4.4/§5.4: the reported range
  // resolution is c/(2B); the GUI example (24000-24245 MHz, B = 245 MHz) gives
  // 0.63 m per bin, so a configured bandwidth always wins over a stale value.
  const double center_hz =
      ParseDouble(params_, "center_frequency_hz", 24.125e9);
  if (center_hz > 0.0) {
    wavelength_m_ = kLightSpeed / center_hz;
  }
  if (bandwidth_hz_ > 0.0) {
    const double implied = kLightSpeed / (2.0 * bandwidth_hz_);
    if (range_resolution_m_ > 0.0 &&
        std::fabs(implied - range_resolution_m_) > 0.1 * implied) {
      AWARN << "ICL1122 bandwidth_hz=" << bandwidth_hz_
            << " implies dR=" << implied << " m, range_resolution_m="
            << range_resolution_m_ << " m; using the bandwidth value";
    }
    range_resolution_m_ = implied;
  }
  AINFO << "ICL1122 scale dR=" << range_resolution_m_
        << " m lambda=" << wavelength_m_ << " m bandwidth_hz=" << bandwidth_hz_
        << " chirp_period_s=" << chirp_period_s_
        << " az_limit_rad=" << max_abs_azimuth_rad_;
}

std::string Icl1122RadarDriver::ResolveRegisterConfigPath() const {
  if (register_config_.empty()) {
    return {};
  }
  std::filesystem::path p(register_config_);
  if (p.is_absolute()) {
    return p.string();
  }
  const auto from_pkg = PackageRoot() / p;
  if (std::filesystem::exists(from_pkg)) {
    return from_pkg.string();
  }
  const auto from_cwd = std::filesystem::current_path() / p;
  if (std::filesystem::exists(from_cwd)) {
    return from_cwd.string();
  }
  return from_pkg.string();
}

bool Icl1122RadarDriver::AfterConnect() {
  if (!host_bringup_) {
    return true;
  }
  const std::string path = ResolveRegisterConfigPath();
  if (path.empty()) {
    AERROR << "ICL1122 host_bringup requires register_config";
    return false;
  }
  protocol::Icl1122HostConfigurator cfg(stream_.get(), path, cmd_response_ms_);
  if (!cfg.Run()) {
    AERROR << "ICL1122 JoySpace host bring-up failed path=" << path;
    return false;
  }
  return true;
}

void Icl1122RadarDriver::OnStopped() {
  target_parser_.Reset();
  mcu_parser_.Reset();
  range_fft_.Reset();
}

void Icl1122RadarDriver::OnBytes(const std::uint8_t* data, std::size_t n) {
  if (data == nullptr || n == 0) {
    return;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t b = data[i];
    if (auto frame = target_parser_.Feed(b)) {
      EmitTargetCloud(*frame);
    }
    if (auto mcu = mcu_parser_.Feed(b)) {
      if (auto sub = protocol::ParseIcl1122RangeFftPayload(mcu->payload)) {
        if (auto fft = range_fft_.Feed(*sub)) {
          EmitRangeFftCloud(*fft);
        }
      }
    }
  }
}

void Icl1122RadarDriver::EmitTargetCloud(
    const protocol::Icl1122TargetFrame& frame) {
  const autolink::Time now = autolink::Time::Now();
  const std::uint64_t stamp_ns = now.ToNanosecond();

  automsgs::msgs::sensor_msgs::RadarScan scan;
  automsgs::msgs::sensor_msgs::RadarTracks tracks;
  automsgs::msgs::sensor_msgs::PointCloud2 cloud;
  lidar::InitXyzitCloud(&cloud, frame_id_);
  std::string& cloud_data = *cloud.mutable_data();
  cloud_data.reserve(frame.targets.size() * lidar::kXyzitPointStep);

  std::uint32_t width = 0;
  std::uint32_t track_i = 0;
  for (const protocol::Icl1122Target& t : frame.targets) {
    if (!t.valid) {
      continue;
    }
    // UM10035 AA FF (x,y): lateral / depth → ground plane X/Y (Z up), matching
    // EVB1122 GUI fan view so distance is readable on the XY grid.
    const float x = static_cast<float>(t.x_m);
    const float y = static_cast<float>(t.y_m);
    const float v = static_cast<float>(t.velocity_mps);
    const float range = std::hypot(x, y);
    const float azimuth = std::atan2(x, y);

    auto* ret = scan.add_returns();
    ret->set_range(range);
    ret->set_azimuth(azimuth);
    ret->set_elevation(0.f);
    ret->set_doppler_velocity(v);
    ret->set_amplitude(std::fabs(v));

    auto* tr = tracks.add_tracks();
    SetTrackUuid(tr->mutable_uuid(), track_i++);
    tr->mutable_position()->set_x(t.x_m);
    tr->mutable_position()->set_y(t.y_m);
    tr->mutable_position()->set_z(0.0);
    tr->mutable_velocity()->set_x(0.0);
    tr->mutable_velocity()->set_y(t.velocity_mps);
    tr->mutable_velocity()->set_z(0.0);
    tr->mutable_acceleration()->set_x(0.0);
    tr->mutable_acceleration()->set_y(0.0);
    tr->mutable_acceleration()->set_z(0.0);
    // Indoor multi-human: small person-scale markers on the fan chart.
    tr->mutable_size()->set_x(0.25);
    tr->mutable_size()->set_y(0.25);
    tr->mutable_size()->set_z(0.25);
    tr->set_classification(
        automsgs::msgs::sensor_msgs::RadarTrack::DYNAMIC);
    FillZeroCov6(tr->mutable_position_covariance());
    FillZeroCov6(tr->mutable_velocity_covariance());
    FillZeroCov6(tr->mutable_acceleration_covariance());
    FillZeroCov6(tr->mutable_size_covariance());

    lidar::AppendXyzitPoint(&cloud_data, x, y, 0.f, std::fabs(v),
                            static_cast<double>(stamp_ns));
    ++width;
  }
  lidar::FinishXyzitCloud(&cloud, width);

  auto sample = std::make_unique<RadarSample>(id_, now, std::move(scan));
  sample->frame_id = frame_id_;
  sample->tracks = std::move(tracks);
  sample->has_tracks = sample->tracks.tracks_size() > 0;
  sample->cloud = std::move(cloud);
  sample->has_cloud = width > 0;
  EmitSample(std::move(sample));
}

void Icl1122RadarDriver::EmitRangeFftCloud(
    const protocol::Icl1122RangeFftFrame& frame) {
  std::vector<float> peak_x;
  std::vector<float> peak_y;
  std::vector<float> peak_i;
  std::vector<std::size_t> peak_bins;
  protocol::ExtractIcl1122RangePeaks(frame.bin_magnitude, range_resolution_m_,
                                     peak_ratio_, skip_near_bins_, &peak_x,
                                     &peak_y, &peak_i, &peak_bins);
  // Indoor Range-FFT: walls/multipath → many CFAR ghosts; keep strongest.
  protocol::KeepTopIcl1122Peaks(max_peaks_, min_peak_frac_, &peak_x, &peak_y,
                                &peak_i, &peak_bins);

  std::vector<float> cloud_x = peak_x;
  std::vector<float> cloud_y = peak_y;
  std::vector<float> cloud_i = peak_i;
  if (publish_spectrum_) {
    protocol::ExtractIcl1122RangeSpectrum(
        frame.bin_magnitude, range_resolution_m_, skip_near_bins_, &cloud_x,
        &cloud_y, &cloud_i);
  }

  const autolink::Time now = autolink::Time::Now();
  const std::uint64_t stamp_ns = now.ToNanosecond();

  // UM10034P §2.3: the 6 dBi beam spans 86° x 28° (H x E); the H plane covers

  // ±43°, so returns outside that cone are grating lobes / multipath.

  std::vector<std::size_t> kept;

  kept.reserve(peak_y.size());

  for (std::size_t k = 0; k < kept.size(); ++k) {

    const std::size_t i = kept[k];

    const float az = protocol::EstimateIcl1122AzimuthFromIq(

        frame.iq_bytes, peak_bins[i], frame.range_bins, frame.num_rx,

        frame.chirp_number, rx_spacing_m_, wavelength_m_);

    // UM10034P §5.3 2DFFT: the chirp-to-chirp phase step is the radial

    // velocity, far finer than a whole-bin frame difference. The frame

    // difference stays as the fallback when the frame period is unknown.

    float doppler = 0.f;

    float quality = 0.f;

    const bool have_doppler =

        doppler_enable_ && doppler_period > 0.0 &&

        protocol::EstimateIcl1122DopplerVelocity(

            frame.iq_bytes, peak_bins[i], frame.range_bins, frame.num_rx,

            frame.chirp_number, doppler_period, wavelength_m_, &doppler,

            &quality);

    const float vel = have_doppler

                          ? static_cast<float>(doppler * doppler_sign_)

                          : (k == 0 ? primary_vel : 0.f);

    auto* ret = scan.add_returns();
    ret->set_range(peak_y[i]);
    ret->set_azimuth(az);
    ret->set_elevation(0.f);
    ret->set_doppler_velocity(vel);
    ret->set_amplitude(peak_i[i]);

    // Lateral from range * sin(az); forward = range * cos(az).
    const float x = peak_y[i] * std::sin(az);
    const float y = peak_y[i] * std::cos(az);
    peak_x[i] = x;

    auto* tr = tracks.add_tracks();
    SetTrackUuid(tr->mutable_uuid(), static_cast<std::uint32_t>(i));
    tr->mutable_position()->set_x(x);
    tr->mutable_position()->set_y(y);
    tr->mutable_position()->set_z(0.0);
    tr->mutable_velocity()->set_x(0.0);
    tr->mutable_velocity()->set_y(static_cast<double>(vel));
    tr->mutable_velocity()->set_z(0.0);
    tr->mutable_acceleration()->set_x(0.0);
    tr->mutable_acceleration()->set_y(0.0);
    tr->mutable_acceleration()->set_z(0.0);
    tr->mutable_size()->set_x(0.2);
    tr->mutable_size()->set_y(0.2);
    tr->mutable_size()->set_z(0.2);
    tr->set_classification(
        automsgs::msgs::sensor_msgs::RadarTrack::DYNAMIC);
    FillZeroCov6(tr->mutable_position_covariance());
    FillZeroCov6(tr->mutable_velocity_covariance());
    FillZeroCov6(tr->mutable_acceleration_covariance());
    FillZeroCov6(tr->mutable_size_covariance());
  }

  automsgs::msgs::sensor_msgs::PointCloud2 cloud;

  lidar::InitXyzitCloud(&cloud, frame_id_);

  std::string& cloud_data = *cloud.mutable_data();

  if (publish_spectrum_) {

    cloud_data.reserve(cloud_y.size() * lidar::kXyzitPointStep);

    for (std::size_t i = 0; i < cloud_y.size(); ++i) {

      lidar::AppendXyzitPoint(&cloud_data, cloud_x[i], cloud_y[i], 0.f,

                              cloud_i[i], static_cast<double>(stamp_ns));

    }

    lidar::FinishXyzitCloud(&cloud,

                            static_cast<std::uint32_t>(cloud_y.size()));

  } else {

    cloud_data.reserve(kept.size() * lidar::kXyzitPointStep);

    for (std::size_t k = 0; k < kept.size(); ++k) {

      const std::size_t i = kept[k];

      lidar::AppendXyzitPoint(&cloud_data, peak_x[i], peak_y[i], 0.f,

                              peak_i[i], static_cast<double>(stamp_ns));

    }

    lidar::FinishXyzitCloud(&cloud, static_cast<std::uint32_t>(kept.size()));

  }


  auto sample = std::make_unique<RadarSample>(id_, now, std::move(scan));
  sample->frame_id = frame_id_;
  sample->tracks = std::move(tracks);
  sample->has_tracks = sample->tracks.tracks_size() > 0;
  sample->cloud = std::move(cloud);
  sample->has_cloud = !cloud_y.empty();
  EmitSample(std::move(sample));
}

SensorDriver* CreateIcl1122RadarDriver(const SensorId& id,
                                       const DriverParams& params) {
  AINFO << "ICL1122/EVB1122 radar driver id=" << id
        << " device=" << GetString(params, "device", "/dev/ttyACM0")
        << " baud=" << ParseInt(params, "baud", 921600)
        << " host_bringup="
        << (ParseBool(params, "host_bringup",
                      !GetString(params, "register_config").empty())
                ? "true"
                : "false");
  return new Icl1122RadarDriver(id, params);
}

}  // namespace hardware
}  // namespace autodriver

REGISTER_RADAR_BACKEND(icl1122, "icl1122",
                       autodriver::hardware::CreateIcl1122RadarDriver,
                       "evb1122", "um10034", "iclegend");
