/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/fathom/projection/point_cloud.hpp"

#include <cmath>
#include <limits>

namespace autonomy {
namespace perception {
namespace fathom {

namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool HasValidIntrinsics(const CameraIntrinsics& intrinsics) {
    return std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) &&
           std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy) &&
           intrinsics.fx > 0.0F && intrinsics.fy > 0.0F;
}

}  // namespace

bool ProjectDepth(const cv::Mat& depth_m, const cv::Mat& mask,
                  const CameraIntrinsics& intrinsics, cv::Mat* xyz,
                  std::string* error) {
    if (xyz == nullptr) {
        SetError(error, "Fathom XYZ output is null.");
        return false;
    }
    if (depth_m.empty() || mask.empty()) {
        SetError(error, "Fathom depth and validity mask must not be empty.");
        return false;
    }
    if (depth_m.type() != CV_32FC1 || mask.type() != CV_8UC1) {
        SetError(error,
                 "Fathom projection requires CV_32FC1 depth and CV_8UC1 mask.");
        return false;
    }
    if (depth_m.size() != mask.size()) {
        SetError(error, "Fathom depth and validity mask dimensions must match.");
        return false;
    }
    if (!HasValidIntrinsics(intrinsics)) {
        SetError(error,
                 "Fathom camera intrinsics must be finite with positive focal "
                 "lengths.");
        return false;
    }

    xyz->create(depth_m.size(), CV_32FC3);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int row = 0; row < depth_m.rows; ++row) {
        const float* depth_row = depth_m.ptr<float>(row);
        const uint8_t* mask_row = mask.ptr<uint8_t>(row);
        cv::Vec3f* xyz_row = xyz->ptr<cv::Vec3f>(row);
        for (int col = 0; col < depth_m.cols; ++col) {
            const float z = depth_row[col];
            if (mask_row[col] == 0 || !std::isfinite(z) || z <= 0.0F) {
                xyz_row[col] = cv::Vec3f(nan, nan, nan);
                continue;
            }
            xyz_row[col] = cv::Vec3f(
                (static_cast<float>(col) - intrinsics.cx) * z / intrinsics.fx,
                (static_cast<float>(row) - intrinsics.cy) * z / intrinsics.fy,
                z);
        }
    }
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
