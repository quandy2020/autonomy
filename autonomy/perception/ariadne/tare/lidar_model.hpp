/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <cmath>
#include <vector>

namespace autonomy::perception::exploration {

// Spherical-bin sensor model (TARE LiDARModel, simplified 2.5D).
class LidarModel {
 public:
  void Configure(double horizontal_resolution_deg, double vertical_resolution_deg,
                 double max_range_m);

  void Reset();
  void MarkOccluded(double azimuth_rad, double elevation_rad, double range_m);
  bool IsVisible(double azimuth_rad, double elevation_rad,
                 double range_m) const;

  int CountVisibleUnknown(double vp_x, double vp_y, double vp_z, double vp_yaw,
                          const std::vector<double>& unknown_x,
                          const std::vector<double>& unknown_y,
                          const std::vector<double>& unknown_z) const;

 private:
  int AzimuthBin(double azimuth_rad) const;
  int ElevationBin(double elevation_rad) const;
  int BinIndex(int az, int el) const;

  double h_res_rad_{0.034906585};
  double v_res_rad_{0.034906585};
  double max_range_m_{8.0};
  int h_bins_{0};
  int v_bins_{0};
  std::vector<float> range_bins_;
};

}  // namespace autonomy::perception::exploration
