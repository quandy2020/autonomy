/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/base/tasks/depth/moge.hpp"

#include "autonomy/common/network/network.hpp"
#include "autonomy/perception/base/options.hpp"
#include "autonomy/perception/base/tasks/task.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace base {
namespace depth {
namespace moge {
namespace {

bool ImageToBgr(const automsgs::msgs::sensor_msgs::Image& rgb, cv::Mat* bgr,
                std::string* error) {
    if (bgr == nullptr) {
        SetTaskError(error, "bgr must not be null.");
        return false;
    }
    const int height = static_cast<int>(rgb.height());
    const int width = static_cast<int>(rgb.width());
    if (height <= 0 || width <= 0) {
        SetTaskError(error, "RGB image has invalid size.");
        return false;
    }
    const size_t expected = static_cast<size_t>(height) * width * 3U;
    if (rgb.data().size() < expected) {
        SetTaskError(error, "RGB buffer is shorter than width*height*3.");
        return false;
    }
    cv::Mat view(height, width, CV_8UC3,
                 const_cast<char*>(rgb.data().data()), rgb.step());
    if (rgb.encoding() == "bgr8") {
        *bgr = view.clone();
        return true;
    }
    if (rgb.encoding() == "rgb8") {
        cv::cvtColor(view, *bgr, cv::COLOR_RGB2BGR);
        return true;
    }
    SetTaskError(error, "RGB encoding must be rgb8 or bgr8.");
    return false;
}

const common::network::ModelTensorInfo* FindInfo(
    const std::vector<common::network::ModelTensorInfo>& infos,
    const char* name) {
    for (const auto& info : infos) {
        if (info.name == name) {
            return &info;
        }
    }
    return nullptr;
}

const common::network::Tensor* FindOutput(
    const common::network::TensorMap& outputs, const char* name) {
    const auto it = outputs.find(name);
    if (it == outputs.end()) {
        return nullptr;
    }
    return &it->second;
}

bool ViewFloat(const common::network::Tensor& tensor, const float** data,
               size_t* count, std::string* error) {
    std::string view_error;
    if (!tensor.TryViewFloat32(data, count, &view_error) || *data == nullptr) {
        SetTaskError(error, "tensor is not float32: " + view_error);
        return false;
    }
    return true;
}

bool AsFloatHw(const common::network::ModelTensorInfo& info,
               const common::network::Tensor& tensor, int* height, int* width,
               const float** data, std::string* error) {
    size_t count = 0;
    if (!ViewFloat(tensor, data, &count, error)) {
        return false;
    }
    const auto& shape = info.shape.Dims();
    if (shape.size() == 2) {
        *height = static_cast<int>(shape[0]);
        *width = static_cast<int>(shape[1]);
    } else if (shape.size() == 3 && shape[0] == 1) {
        *height = static_cast<int>(shape[1]);
        *width = static_cast<int>(shape[2]);
    } else if (shape.size() == 4 && shape[0] == 1 && shape[1] == 1) {
        *height = static_cast<int>(shape[2]);
        *width = static_cast<int>(shape[3]);
    } else if (shape.size() >= 2 && shape[shape.size() - 1] > 0 &&
               shape[shape.size() - 2] > 0) {
        *height = static_cast<int>(shape[shape.size() - 2]);
        *width = static_cast<int>(shape[shape.size() - 1]);
    } else {
        SetTaskError(error, "mask tensor shape is not [H,W] or [1,H,W].");
        return false;
    }
    return true;
}

bool AsFloatPoints(const common::network::ModelTensorInfo& info,
                   const common::network::Tensor& tensor, int* height,
                   int* width, std::vector<float>* nhwc, std::string* error) {
    const float* raw = nullptr;
    size_t count = 0;
    if (!ViewFloat(tensor, &raw, &count, error)) {
        return false;
    }
    const auto& shape = info.shape.Dims();
    if (shape.size() == 4 && shape[0] == 1 && shape[3] == 3) {
        *height = static_cast<int>(shape[1]);
        *width = static_cast<int>(shape[2]);
        nhwc->assign(raw, raw + static_cast<size_t>(*height) * (*width) * 3);
        return true;
    }
    if (shape.size() == 4 && shape[0] == 1 && shape[1] == 3) {
        *height = static_cast<int>(shape[2]);
        *width = static_cast<int>(shape[3]);
        nhwc->resize(static_cast<size_t>(*height) * (*width) * 3);
        const size_t plane = static_cast<size_t>(*height) * (*width);
        for (size_t i = 0; i < plane; ++i) {
            (*nhwc)[i * 3 + 0] = raw[0 * plane + i];
            (*nhwc)[i * 3 + 1] = raw[1 * plane + i];
            (*nhwc)[i * 3 + 2] = raw[2 * plane + i];
        }
        return true;
    }
    if (shape.size() == 3 && shape[2] == 3) {
        *height = static_cast<int>(shape[0]);
        *width = static_cast<int>(shape[1]);
        nhwc->assign(raw, raw + static_cast<size_t>(*height) * (*width) * 3);
        return true;
    }
    SetTaskError(error, "points tensor shape is not [1,H,W,3] or [1,3,H,W].");
    return false;
}

float MetricScale(const common::network::Tensor& tensor) {
    const float* raw = nullptr;
    size_t count = 0;
    std::string view_error;
    if (!tensor.TryViewFloat32(&raw, &count, &view_error) || raw == nullptr ||
        count == 0) {
        return 1.0F;
    }
    return std::max(raw[0], 1e-6F);
}

// Closed-form focal/shift recovery (MoGe point_map_to_depth_legacy).
bool RecoverDepth(const std::vector<float>& points_nhwc, int height, int width,
                  const float* mask, float mask_threshold, float metric_scale,
                  float fov_x_deg, cv::Mat* depth_m, std::string* error) {
    if (height <= 1 || width <= 1) {
        SetTaskError(error, "MoGe points map is empty.");
        return false;
    }
    const float aspect =
        static_cast<float>(width) / static_cast<float>(height);
    const float span_x = aspect / std::sqrt(1.0F + aspect * aspect);
    const float span_y = 1.0F / std::sqrt(1.0F + aspect * aspect);

    double a00 = 0.0;
    double a01 = 0.0;
    double a11 = 0.0;
    double b0 = 0.0;
    double b1 = 0.0;
    int count = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t i = static_cast<size_t>(y) * width + x;
            if (mask != nullptr && mask[i] < mask_threshold) {
                continue;
            }
            const float px = points_nhwc[i * 3 + 0];
            const float py = points_nhwc[i * 3 + 1];
            const float pz = points_nhwc[i * 3 + 2];
            if (!std::isfinite(px) || !std::isfinite(py) ||
                !std::isfinite(pz)) {
                continue;
            }
            const float uu =
                (width <= 1)
                    ? 0.0F
                    : (-span_x * static_cast<float>(width - 1) / width) +
                          (2.0F * span_x * static_cast<float>(width - 1) /
                           width) *
                              (static_cast<float>(x) /
                               static_cast<float>(width - 1));
            const float vv =
                (height <= 1)
                    ? 0.0F
                    : (-span_y * static_cast<float>(height - 1) / height) +
                          (2.0F * span_y * static_cast<float>(height - 1) /
                           height) *
                              (static_cast<float>(y) /
                               static_cast<float>(height - 1));
            a00 += static_cast<double>(px) * px + static_cast<double>(py) * py;
            a01 += -static_cast<double>(uu) * px - static_cast<double>(vv) * py;
            a11 += static_cast<double>(uu) * uu + static_cast<double>(vv) * vv;
            b0 += static_cast<double>(uu) * pz * px +
                  static_cast<double>(vv) * pz * py;
            b1 += -static_cast<double>(uu) * uu * pz -
                  static_cast<double>(vv) * vv * pz;
            ++count;
        }
    }
    if (count < 16) {
        SetTaskError(error, "MoGe has too few valid points for depth recovery.");
        return false;
    }

    float focal = 1.0F;
    float shift = 0.0F;
    if (fov_x_deg > 0.0F) {
        const float fov = fov_x_deg * static_cast<float>(M_PI) / 180.0F;
        focal = aspect / std::sqrt(1.0F + aspect * aspect) /
                std::tan(0.5F * fov);
        // With known focal, solve only for shift via 1D projection residual.
        double num = 0.0;
        double den = 0.0;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const size_t i = static_cast<size_t>(y) * width + x;
                if (mask != nullptr && mask[i] < mask_threshold) {
                    continue;
                }
                const float px = points_nhwc[i * 3 + 0];
                const float py = points_nhwc[i * 3 + 1];
                const float pz = points_nhwc[i * 3 + 2];
                const float uu =
                    (width <= 1)
                        ? 0.0F
                        : (-span_x * static_cast<float>(width - 1) / width) +
                              (2.0F * span_x * static_cast<float>(width - 1) /
                               width) *
                                  (static_cast<float>(x) /
                                   static_cast<float>(width - 1));
                const float vv =
                    (height <= 1)
                        ? 0.0F
                        : (-span_y * static_cast<float>(height - 1) / height) +
                              (2.0F * span_y * static_cast<float>(height - 1) /
                               height) *
                                  (static_cast<float>(y) /
                                   static_cast<float>(height - 1));
                const double rxy =
                    std::sqrt(static_cast<double>(px) * px +
                              static_cast<double>(py) * py);
                const double ruv =
                    std::sqrt(static_cast<double>(uu) * uu +
                              static_cast<double>(vv) * vv);
                if (rxy < 1e-6 || ruv < 1e-6) {
                    continue;
                }
                const double target_z =
                    focal * rxy / std::max(ruv, 1e-6);
                num += target_z - pz;
                den += 1.0;
            }
        }
        shift = den > 0.0 ? static_cast<float>(num / den) : 0.0F;
        (void)a00;
        (void)a01;
        (void)a11;
        (void)b0;
        (void)b1;
    } else {
        a00 += 1e-6;
        a11 += 1e-6;
        const double det = a00 * a11 - a01 * a01;
        if (std::abs(det) < 1e-12) {
            SetTaskError(error, "MoGe focal/shift solve is singular.");
            return false;
        }
        focal = static_cast<float>((a11 * b0 - a01 * b1) / det);
        shift = static_cast<float>((-a01 * b0 + a00 * b1) / det);
        if (!(focal > 1e-4F)) {
            SetTaskError(error, "MoGe recovered non-positive focal.");
            return false;
        }
    }

    *depth_m = cv::Mat(height, width, CV_32FC1);
    for (int y = 0; y < height; ++y) {
        float* row = depth_m->ptr<float>(y);
        for (int x = 0; x < width; ++x) {
            const size_t i = static_cast<size_t>(y) * width + x;
            if (mask != nullptr && mask[i] < mask_threshold) {
                row[x] = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            const float z =
                (points_nhwc[i * 3 + 2] + shift) * metric_scale;
            row[x] = (std::isfinite(z) && z > 0.0F)
                         ? z
                         : std::numeric_limits<float>::quiet_NaN();
        }
    }
    return true;
}

void FillDepthMessage(const cv::Mat& depth_m,
                      const automsgs::msgs::sensor_msgs::Image& rgb,
                      const std::string& frame_id,
                      automsgs::msgs::sensor_msgs::Image* out) {
    out->mutable_header()->CopyFrom(rgb.header());
    if (!frame_id.empty()) {
        out->mutable_header()->set_frame_id(frame_id);
    }
    out->set_height(static_cast<uint32_t>(depth_m.rows));
    out->set_width(static_cast<uint32_t>(depth_m.cols));
    out->set_encoding("32FC1");
    out->set_is_bigendian(false);
    out->set_step(static_cast<uint32_t>(depth_m.cols * sizeof(float)));
    out->set_data(reinterpret_cast<const char*>(depth_m.ptr<float>(0)),
                  static_cast<size_t>(depth_m.rows) * depth_m.cols *
                      sizeof(float));
}

}  // namespace

bool Decode(const common::network::Engine& engine,
            const proto::BaseOptions& options,
            const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
            std::string* error) {
    if (outputs == nullptr) {
        SetTaskError(error, "outputs must not be null.");
        return false;
    }
    cv::Mat bgr;
    if (!ImageToBgr(rgb, &bgr, error)) {
        return false;
    }

    const int model_h = static_cast<int>(options.input_height());
    const int model_w = static_cast<int>(options.input_width());
    common::network::PreprocessOptions prep =
        common::network::Stretch(model_h, model_w);
    prep.swap_red_blue = true;  // BGR mat → RGB network input

    // Engine::Run is non-const; MoGe decode owns a const view from Model.
    auto* mutable_engine = const_cast<common::network::Engine*>(&engine);
    common::network::RunResult result;
    std::string run_error;
    if (!common::network::RunPipeline(mutable_engine, bgr, prep, &result,
                                      &run_error)) {
        SetTaskError(error, "MoGe inference failed: " + run_error);
        return false;
    }

    const auto* points_t = FindOutput(result.outputs, "points");
    const auto* mask_t = FindOutput(result.outputs, "mask");
    const auto* scale_t = FindOutput(result.outputs, "metric_scale");
    if (points_t == nullptr) {
        SetTaskError(error, "MoGe output 'points' is missing.");
        return false;
    }
    const auto infos = engine.GetOutputInfos();
    const auto* points_info = FindInfo(infos, "points");
    if (points_info == nullptr) {
        SetTaskError(error, "MoGe metadata for 'points' is missing.");
        return false;
    }

    int ph = 0;
    int pw = 0;
    std::vector<float> points_nhwc;
    if (!AsFloatPoints(*points_info, *points_t, &ph, &pw, &points_nhwc,
                       error)) {
        return false;
    }

    const float* mask = nullptr;
    if (mask_t != nullptr) {
        const auto* mask_info = FindInfo(infos, "mask");
        if (mask_info == nullptr) {
            SetTaskError(error, "MoGe metadata for 'mask' is missing.");
            return false;
        }
        int mh = 0;
        int mw = 0;
        if (!AsFloatHw(*mask_info, *mask_t, &mh, &mw, &mask, error)) {
            return false;
        }
        if (mh != ph || mw != pw) {
            SetTaskError(error, "MoGe mask size does not match points.");
            return false;
        }
    }

    const float metric_scale =
        scale_t != nullptr ? MetricScale(*scale_t) : 1.0F;
    const auto* depth_task = FindTask(options, proto::TASK_DEPTH);
    const float mask_thr =
        (depth_task != nullptr && depth_task->moge_mask_threshold() > 0.0F)
            ? depth_task->moge_mask_threshold()
            : 0.5F;
    const float fov_x_deg =
        depth_task != nullptr ? depth_task->moge_fov_x_deg() : 0.0F;

    cv::Mat depth_model;
    if (!RecoverDepth(points_nhwc, ph, pw, mask, mask_thr, metric_scale,
                      fov_x_deg, &depth_model, error)) {
        return false;
    }

    cv::Mat depth_full;
    if (depth_model.cols != bgr.cols || depth_model.rows != bgr.rows) {
        cv::resize(depth_model, depth_full, bgr.size(), 0, 0, cv::INTER_LINEAR);
    } else {
        depth_full = depth_model;
    }

    FillDepthMessage(depth_full, rgb, options.camera_frame(), &outputs->depth);
    return true;
}

}  // namespace moge
}  // namespace depth
}  // namespace base
}  // namespace perception
}  // namespace autonomy
