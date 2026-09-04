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

/**
 * @file
 * @brief Depth image resize and metre normalization.
 */

#include "autonomy/perception/track/utils/depth_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/track/common/constants.hpp"

namespace autonomy::perception::track {
namespace {

enum class DepthEncoding {
  kUnsupported,
  kUint16Millimetres,
  kFloat32Metres,
};

DepthEncoding ClassifyDepthEncoding(const std::string& encoding) {
  if (encoding == "16UC1" || encoding == "mono16") {
    return DepthEncoding::kUint16Millimetres;
  }
  if (encoding == "32FC1") {
    return DepthEncoding::kFloat32Metres;
  }
  return DepthEncoding::kUnsupported;
}

float ReadDepthMetres(const automsgs::msgs::sensor_msgs::Image& image, int x,
                      int y, DepthEncoding encoding,
                      double depth_scale_to_metres) {
  x = std::clamp(x, 0, static_cast<int>(image.width()) - 1);
  y = std::clamp(y, 0, static_cast<int>(image.height()) - 1);
  const size_t index =
      static_cast<size_t>(y) * static_cast<size_t>(image.width()) +
      static_cast<size_t>(x);
  const auto& bytes = image.data();

  if (encoding == DepthEncoding::kUint16Millimetres) {
    if ((index + 1) * 2 > bytes.size()) {
      return 0.0f;
    }
    uint16_t raw = 0;
    std::memcpy(&raw, bytes.data() + index * 2, sizeof(raw));
    return static_cast<float>(raw) *
           static_cast<float>(depth_scale_to_metres);
  }

  // 32FC1 is already metres; do not apply millimetre scale.
  if ((index + 1) * 4 > bytes.size()) {
    return 0.0f;
  }
  float raw = 0.0f;
  std::memcpy(&raw, bytes.data() + index * 4, sizeof(raw));
  return raw;
}

}  // namespace

bool NormalizeDepthImage(const automsgs::msgs::sensor_msgs::Image& image,
                         int output_height, int output_width,
                         double depth_scale_to_metres, double max_depth_m,
                         std::vector<float>* normalized_depth) {
  if (normalized_depth == nullptr || output_height <= 0 || output_width <= 0) {
    return false;
  }
  normalized_depth->assign(
      static_cast<size_t>(output_height * output_width), 0.0f);
  if (image.width() == 0 || image.height() == 0 || image.data().empty()) {
    return false;
  }

  const DepthEncoding encoding = ClassifyDepthEncoding(image.encoding());
  if (encoding == DepthEncoding::kUnsupported) {
    AWARN << "NormalizeDepthImage: unsupported encoding '" << image.encoding()
          << "'";
    return false;
  }

  const double scale = depth_scale_to_metres > 0.0
                           ? depth_scale_to_metres
                           : defaults::kDepthScaleToMetres;
  const float max_depth =
      max_depth_m > 0.0
          ? static_cast<float>(max_depth_m)
          : static_cast<float>(defaults::kPlanningHorizonM *
                               kDepthNormalizeHorizonFactor);

  for (int out_row = 0; out_row < output_height; ++out_row) {
    const int source_row =
        out_row * static_cast<int>(image.height()) / output_height;
    for (int out_col = 0; out_col < output_width; ++out_col) {
      const int source_col =
          out_col * static_cast<int>(image.width()) / output_width;
      float depth_m =
          ReadDepthMetres(image, source_col, source_row, encoding, scale);
      if (!(depth_m > 0.0f) || !std::isfinite(depth_m)) {
        depth_m = max_depth;
      }
      depth_m = std::min(depth_m, max_depth) / max_depth;
      (*normalized_depth)[static_cast<size_t>(out_row * output_width +
                                               out_col)] = depth_m;
    }
  }
  return true;
}

}  // namespace autonomy::perception::track
