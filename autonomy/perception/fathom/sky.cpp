/*
 * Copyright 2026 The OpenRobotic Beginner Authors
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

#include "autonomy/perception/fathom/sky.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace autonomy {
namespace perception {
namespace fathom {

proto::SkyCorrectOptions DefaultSkyCorrectOptions() {
  proto::SkyCorrectOptions options;
  options.set_enabled(true);
  options.set_top_frac(0.55F);
  options.set_sat_max(60);
  options.set_val_min(130);
  options.set_tex_max(12.0F);
  options.set_far_floor(20.0F);
  options.set_min_far_area(1024);
  options.set_near_thresh(3.0F);
  options.set_near_grow(10.0F);
  options.set_min_area(1024);
  options.set_fallback_far(50.0F);
  return options;
}

namespace {

proto::SkyCorrectOptions WithDefaults(const proto::SkyCorrectOptions& in) {
  proto::SkyCorrectOptions options = DefaultSkyCorrectOptions();
  options.set_enabled(in.enabled());
  if (in.top_frac() > 0.0F) {
    options.set_top_frac(in.top_frac());
  }
  if (in.sat_max() > 0) {
    options.set_sat_max(in.sat_max());
  }
  if (in.val_min() > 0) {
    options.set_val_min(in.val_min());
  }
  if (in.tex_max() > 0.0F) {
    options.set_tex_max(in.tex_max());
  }
  if (in.far_floor() > 0.0F) {
    options.set_far_floor(in.far_floor());
  }
  if (in.min_far_area() > 0) {
    options.set_min_far_area(in.min_far_area());
  }
  if (in.near_thresh() > 0.0F) {
    options.set_near_thresh(in.near_thresh());
  }
  if (in.near_grow() > 0.0F) {
    options.set_near_grow(in.near_grow());
  }
  if (in.min_area() > 0) {
    options.set_min_area(in.min_area());
  }
  if (in.fallback_far() > 0.0F) {
    options.set_fallback_far(in.fallback_far());
  }
  return options;
}

}  // namespace

cv::Mat SkyAppearanceMask(const cv::Mat& bgr,
                          const proto::SkyCorrectOptions& raw_options) {
  const proto::SkyCorrectOptions opt = WithDefaults(raw_options);
  const int h = bgr.rows;
  const int w = bgr.cols;

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat channels[3];
  cv::split(hsv, channels);
  const cv::Mat low_sat = channels[1] < opt.sat_max();
  const cv::Mat high_val = channels[2] > opt.val_min();

  cv::Mat gray;
  cv::Mat gx;
  cv::Mat gy;
  cv::Mat tex;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
  cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
  cv::magnitude(gx, gy, tex);
  cv::blur(tex, tex, cv::Size(5, 5));
  const cv::Mat low_tex = tex < opt.tex_max();

  cv::Mat appear = cv::Mat::zeros(h, w, CV_8UC1);
  const int top_rows =
      std::clamp(static_cast<int>(opt.top_frac() * static_cast<float>(h)), 1,
                 h);
  cv::Mat roi = appear(cv::Rect(0, 0, w, top_rows));
  cv::bitwise_and(low_sat(cv::Rect(0, 0, w, top_rows)),
                  high_val(cv::Rect(0, 0, w, top_rows)), roi);
  cv::bitwise_and(roi, low_tex(cv::Rect(0, 0, w, top_rows)), roi);

  cv::Mat labels;
  const int n = cv::connectedComponents(appear, labels, 8, CV_32S);
  if (n <= 1) {
    return cv::Mat::zeros(h, w, CV_8UC1);
  }
  std::vector<uint8_t> top_label(static_cast<size_t>(n), 0);
  const int probe_rows = std::min(3, h);
  for (int y = 0; y < probe_rows; ++y) {
    const int* lab = labels.ptr<int>(y);
    const uint8_t* ap = appear.ptr<uint8_t>(y);
    for (int x = 0; x < w; ++x) {
      if (ap[x] != 0) {
        top_label[static_cast<size_t>(lab[x])] = 1;
      }
    }
  }
  cv::Mat keep = cv::Mat::zeros(h, w, CV_8UC1);
  for (int y = 0; y < h; ++y) {
    const int* lab = labels.ptr<int>(y);
    uint8_t* kp = keep.ptr<uint8_t>(y);
    for (int x = 0; x < w; ++x) {
      const int label = lab[x];
      if (label != 0 && top_label[static_cast<size_t>(label)] != 0) {
        kp[x] = 255;
      }
    }
  }
  return keep;
}

int CorrectSkyFar(cv::Mat& pred, const cv::Mat& bgr,
                  const proto::SkyCorrectOptions& raw_options,
                  cv::Mat* detect_out) {
  const proto::SkyCorrectOptions opt = WithDefaults(raw_options);
  CV_Assert(pred.type() == CV_32FC1);
  const int h = pred.rows;
  const int w = pred.cols;

  const cv::Mat sky = SkyAppearanceMask(bgr, opt);
  if (detect_out != nullptr) {
    *detect_out = cv::Mat::zeros(h, w, CV_8UC1);
  }
  if (cv::countNonZero(sky) == 0) {
    return 0;
  }

  int far_evidence = 0;
  for (int y = 0; y < h; ++y) {
    const uint8_t* sk = sky.ptr<uint8_t>(y);
    const float* pr = pred.ptr<float>(y);
    for (int x = 0; x < w; ++x) {
      if (sk[x] != 0 && std::isfinite(pr[x]) && pr[x] >= opt.far_floor()) {
        ++far_evidence;
      }
    }
  }
  if (far_evidence < opt.min_far_area()) {
    return 0;
  }

  cv::Mat seed(h, w, CV_8UC1);
  cv::Mat weak(h, w, CV_8UC1);
  for (int y = 0; y < h; ++y) {
    const uint8_t* sk = sky.ptr<uint8_t>(y);
    const float* pr = pred.ptr<float>(y);
    uint8_t* sd = seed.ptr<uint8_t>(y);
    uint8_t* wk = weak.ptr<uint8_t>(y);
    for (int x = 0; x < w; ++x) {
      const bool in_sky = sk[x] != 0 && std::isfinite(pr[x]);
      sd[x] = (in_sky && pr[x] < opt.near_thresh()) ? 255 : 0;
      wk[x] = (in_sky && pr[x] < opt.near_grow()) ? 255 : 0;
    }
  }

  cv::Mat labels;
  const int n = cv::connectedComponents(weak, labels, 8, CV_32S);
  if (n <= 1) {
    return 0;
  }

  std::vector<int> seed_count(static_cast<size_t>(n), 0);
  for (int y = 0; y < h; ++y) {
    const int* lab = labels.ptr<int>(y);
    const uint8_t* sd = seed.ptr<uint8_t>(y);
    for (int x = 0; x < w; ++x) {
      if (sd[x] != 0) {
        ++seed_count[static_cast<size_t>(lab[x])];
      }
    }
  }
  std::vector<uint8_t> is_anomaly(static_cast<size_t>(n), 0);
  bool any = false;
  for (int label = 1; label < n; ++label) {
    if (seed_count[static_cast<size_t>(label)] >= opt.min_area()) {
      is_anomaly[static_cast<size_t>(label)] = 1;
      any = true;
    }
  }
  if (!any) {
    return 0;
  }

  std::vector<float> far_vals;
  far_vals.reserve(static_cast<size_t>(far_evidence));
  for (int y = 0; y < h; ++y) {
    const uint8_t* sk = sky.ptr<uint8_t>(y);
    const float* pr = pred.ptr<float>(y);
    for (int x = 0; x < w; ++x) {
      if (sk[x] != 0 && std::isfinite(pr[x]) && pr[x] >= opt.near_grow()) {
        far_vals.push_back(pr[x]);
      }
    }
  }
  float far_val = opt.fallback_far();
  if (!far_vals.empty()) {
    const size_t mid = far_vals.size() / 2;
    std::nth_element(far_vals.begin(), far_vals.begin() + static_cast<std::ptrdiff_t>(mid),
                     far_vals.end());
    far_val = far_vals[mid];
  }
  far_val = std::max(far_val, opt.near_grow());

  int fixed = 0;
  for (int y = 0; y < h; ++y) {
    const int* lab = labels.ptr<int>(y);
    float* pr = pred.ptr<float>(y);
    uint8_t* det =
        detect_out != nullptr ? detect_out->ptr<uint8_t>(y) : nullptr;
    for (int x = 0; x < w; ++x) {
      if (is_anomaly[static_cast<size_t>(lab[x])] != 0) {
        pr[x] = far_val;
        if (det != nullptr) {
          det[x] = 255;
        }
        ++fixed;
      }
    }
  }
  return fixed;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
