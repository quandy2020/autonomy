/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/lidar_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy::perception::exploration {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

void LidarModel::Configure(double horizontal_resolution_deg,
                           double vertical_resolution_deg, double max_range_m) {
  h_res_rad_ = horizontal_resolution_deg * kPi / 180.0;
  v_res_rad_ = vertical_resolution_deg * kPi / 180.0;
  max_range_m_ = max_range_m > 0 ? max_range_m : 8.0;
  h_bins_ = std::max(1, static_cast<int>(std::ceil(2.0 * kPi / h_res_rad_)));
  v_bins_ = std::max(1, static_cast<int>(std::ceil(kPi / v_res_rad_)));
  range_bins_.assign(static_cast<size_t>(h_bins_ * v_bins_),
                     std::numeric_limits<float>::infinity());
}

void LidarModel::Reset() {
  std::fill(range_bins_.begin(), range_bins_.end(),
            std::numeric_limits<float>::infinity());
}

int LidarModel::AzimuthBin(double azimuth_rad) const {
  double wrapped = std::fmod(azimuth_rad + kPi, 2.0 * kPi);
  if (wrapped < 0) {
    wrapped += 2.0 * kPi;
  }
  wrapped -= kPi;
  const int bin = static_cast<int>(std::floor((wrapped + kPi) / h_res_rad_));
  return std::clamp(bin, 0, h_bins_ - 1);
}

int LidarModel::ElevationBin(double elevation_rad) const {
  const double clamped =
      std::clamp(elevation_rad, -kPi / 2.0, kPi / 2.0) + kPi / 2.0;
  const int bin = static_cast<int>(std::floor(clamped / v_res_rad_));
  return std::clamp(bin, 0, v_bins_ - 1);
}

int LidarModel::BinIndex(int az, int el) const {
  return el * h_bins_ + az;
}

void LidarModel::MarkOccluded(double azimuth_rad, double elevation_rad,
                              double range_m) {
  if (range_bins_.empty()) {
    return;
  }
  const int idx = BinIndex(AzimuthBin(azimuth_rad), ElevationBin(elevation_rad));
  range_bins_[static_cast<size_t>(idx)] =
      std::min(range_bins_[static_cast<size_t>(idx)], static_cast<float>(range_m));
}

bool LidarModel::IsVisible(double azimuth_rad, double elevation_rad,
                           double range_m) const {
  if (range_bins_.empty()) {
    return true;
  }
  const int idx = BinIndex(AzimuthBin(azimuth_rad), ElevationBin(elevation_rad));
  return range_m <= range_bins_[static_cast<size_t>(idx)] + 0.05f;
}

int LidarModel::CountVisibleUnknown(
    double vp_x, double vp_y, double vp_z, double vp_yaw,
    const std::vector<double>& unknown_x, const std::vector<double>& unknown_y,
    const std::vector<double>& unknown_z) const {
  std::vector<float> local_bins = range_bins_;
  auto is_visible = [&](double az, double el, double range) {
    if (local_bins.empty()) {
      return true;
    }
    const int idx = BinIndex(AzimuthBin(az), ElevationBin(el));
    return range <= local_bins[static_cast<size_t>(idx)] + 0.05f;
  };
  auto mark = [&](double az, double el, double range) {
    if (local_bins.empty()) {
      return;
    }
    const int idx = BinIndex(AzimuthBin(az), ElevationBin(el));
    local_bins[static_cast<size_t>(idx)] =
        std::min(local_bins[static_cast<size_t>(idx)], static_cast<float>(range));
  };
  int count = 0;
  for (size_t i = 0; i < unknown_x.size(); ++i) {
    const double dx = unknown_x[i] - vp_x;
    const double dy = unknown_y[i] - vp_y;
    const double dz = unknown_z[i] - vp_z;
    const double range = std::hypot(dx, std::hypot(dy, dz));
    if (range > max_range_m_ || range < 0.05) {
      continue;
    }
    const double az = std::atan2(dy, dx) - vp_yaw;
    const double el = std::atan2(dz, std::hypot(dx, dy));
    if (is_visible(az, el, range)) {
      mark(az, el, range);
      ++count;
    }
  }
  return count;
}

}  // namespace autonomy::perception::exploration
