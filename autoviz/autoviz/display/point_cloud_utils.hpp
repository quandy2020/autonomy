/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <QColor>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autoviz {
namespace display {

struct ParsedPointCloud {
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  std::vector<float> intensities;
  std::vector<uint32_t> rgb;
};

enum class PointCloudColorMode { kFlat, kIntensity, kRgb8 };

/** Decode commsgs PointCloud2 into xyz arrays with optional intensity/rgb. */
ParsedPointCloud parsePointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    uint32_t decimation = 1);

QColor colorFromIntensity(float intensity, float min_i, float max_i);
/** 256-entry rainbow table shared with Ogre indexed_8bit palette. */
const std::array<QColor, 256>& intensityRainbowTable();
uint8_t intensityToPaletteIndex(float intensity, float min_i, float max_i);
QColor colorFromIntensityIndex(uint8_t index);
QColor colorFromRgbPacked(uint32_t rgb_packed);

}  // namespace display
}  // namespace autoviz
