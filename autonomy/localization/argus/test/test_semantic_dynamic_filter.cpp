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

#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>

#include "argus/frontend/semantic_dynamic_filter.hpp"
#include "argus/frontend/types.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::frontend::SemanticBackend;
  using argus::frontend::SemanticDynamicFilter;
  using argus::frontend::SemanticDynamicFilterConfig;
  using argus::frontend::StatesGroup;
  using argus::sensor::PointCloudXYZI;
  using argus::sensor::PointType;

  SemanticDynamicFilterConfig cfg;
  cfg.backend = SemanticBackend::kMaskFile;
  cfg.fx = 100.0;
  cfg.fy = 100.0;
  cfg.cx = 50.0;
  cfg.cy = 40.0;
  cfg.R_cl = Eigen::Matrix3d::Identity();
  cfg.t_cl = Eigen::Vector3d::Zero();

  SemanticDynamicFilter filter(cfg);
  if (filter.model_loaded()) {
    return Fail("mask_file backend should not report ONNX model_loaded");
  }

  // 100x80 keep mask with a hole at (50,40)
  cv::Mat keep(80, 100, CV_8UC1, cv::Scalar(1));
  keep.at<uint8_t>(40, 50) = 0;
  filter.SetKeepMask(keep);
  if (!filter.has_keep_mask()) {
    return Fail("SetKeepMask");
  }

  PointCloudXYZI cloud;
  {
    PointType static_pt;
    static_pt.x = 1.0f;  // u = 100*1/10+50 = 60
    static_pt.y = 0.0f;
    static_pt.z = 10.0f;
    cloud.push_back(static_pt);

    PointType dyn;
    dyn.x = 0.0f;  // u=50, v=40 → reject
    dyn.y = 0.0f;
    dyn.z = 10.0f;
    cloud.push_back(dyn);

    PointType behind;
    behind.x = 0.0f;
    behind.y = 0.0f;
    behind.z = -1.0f;  // behind camera → keep
    cloud.push_back(behind);
  }

  StatesGroup state;
  auto mask = filter.FilterLidar(cloud, state);
  if (mask.keep.size() != 3) {
    return Fail("mask size");
  }
  if (mask.keep[0] != 1) {
    return Fail("static point must keep");
  }
  if (mask.keep[1] != 0) {
    return Fail("projected dynamic must reject");
  }
  if (mask.keep[2] != 1) {
    return Fail("behind camera must keep");
  }
  if (std::abs(mask.RejectedRatio() - 1.0 / 3.0) > 1e-6) {
    return Fail("rejected ratio");
  }

  // FilterVisual with empty ONNX → pass-through pixel mask size
  cv::Mat img(80, 100, CV_8UC3, cv::Scalar(10, 10, 10));
  auto vm = filter.FilterVisual(img, state);
  if (vm.keep.size() != static_cast<std::size_t>(80 * 100)) {
    return Fail("FilterVisual size");
  }

  std::cout << "OK semantic_dynamic_filter projection/mask\n";
#ifdef ARGUS_HAS_ONNXRUNTIME
  std::cout << "  (built with ARGUS_HAS_ONNXRUNTIME)\n";
#else
  std::cout << "  (ONNX Runtime not linked; onnx path compile-disabled)\n";
#endif
  return 0;
}
