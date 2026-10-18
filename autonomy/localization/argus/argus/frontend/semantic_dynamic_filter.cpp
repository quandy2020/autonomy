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

#include "argus/frontend/semantic_dynamic_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef ARGUS_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace argus {
namespace frontend {
namespace {

bool IsDynamicClass(int class_id, const std::unordered_set<int>& dyn) {
    return dyn.find(class_id) != dyn.end();
}

cv::Mat KeepFromClassMap(const cv::Mat& class_map,
                         const std::unordered_set<int>& dyn) {
    cv::Mat keep(class_map.rows, class_map.cols, CV_8UC1, cv::Scalar(1));
    for (int r = 0; r < class_map.rows; ++r) {
        const int* row = class_map.ptr<int>(r);
        uint8_t* out = keep.ptr<uint8_t>(r);
        for (int c = 0; c < class_map.cols; ++c) {
            if (IsDynamicClass(row[c], dyn)) {
                out[c] = 0;
            }
        }
    }
    return keep;
}

#ifdef ARGUS_HAS_ONNXRUNTIME
std::vector<float> MakeNchwInput(const cv::Mat& bgr, int H, int W,
                                 const std::vector<float>& mean,
                                 const std::vector<float>& stdv) {
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(W, H), 0, 0, cv::INTER_LINEAR);
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

    std::vector<float> input(static_cast<std::size_t>(3 * H * W));
    for (int y = 0; y < H; ++y) {
        const cv::Vec3f* row = rgb.ptr<cv::Vec3f>(y);
        for (int x = 0; x < W; ++x) {
            for (int c = 0; c < 3; ++c) {
                const float v =
                    (row[x][c] - mean[static_cast<std::size_t>(c)]) /
                    stdv[static_cast<std::size_t>(c)];
                input[static_cast<std::size_t>(c * H * W + y * W + x)] = v;
            }
        }
    }
    return input;
}

cv::Mat ArgmaxLogitsToClassMap(const float* data, int C, int h, int w) {
    cv::Mat class_map(h, w, CV_32SC1);
    for (int y = 0; y < h; ++y) {
        int* row = class_map.ptr<int>(y);
        for (int x = 0; x < w; ++x) {
            int best = 0;
            float best_v = data[0 * h * w + y * w + x];
            for (int c = 1; c < C; ++c) {
                const float v = data[c * h * w + y * w + x];
                if (v > best_v) {
                    best_v = v;
                    best = c;
                }
            }
            row[x] = best;
        }
    }
    return class_map;
}

bool ProbMapToKeep(const float* data, int h, int w, double threshold,
                   cv::Mat* keep_out) {
    *keep_out = cv::Mat(h, w, CV_8UC1);
    for (int y = 0; y < h; ++y) {
        uint8_t* row = keep_out->ptr<uint8_t>(y);
        for (int x = 0; x < w; ++x) {
            row[x] = (data[y * w + x] >= threshold) ? 0 : 1;
        }
    }
    return true;
}

cv::Mat LabelsToClassMap(Ort::Value& out, const std::vector<int64_t>& oshape,
                         ONNXTensorElementDataType et) {
    const int h = static_cast<int>(oshape[oshape.size() - 2]);
    const int w = static_cast<int>(oshape[oshape.size() - 1]);
    cv::Mat class_map(h, w, CV_32SC1);
    if (et == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
        const int64_t* data = out.GetTensorData<int64_t>();
        for (int i = 0; i < h * w; ++i) {
            class_map.at<int>(i / w, i % w) = static_cast<int>(data[i]);
        }
    } else {
        const int32_t* data = out.GetTensorData<int32_t>();
        for (int i = 0; i < h * w; ++i) {
            class_map.at<int>(i / w, i % w) = static_cast<int>(data[i]);
        }
    }
    return class_map;
}

bool DecodeOnnxKeepMask(Ort::Value& out, double mask_threshold,
                        const std::unordered_set<int>& dyn, cv::Mat* keep_out) {
    auto info = out.GetTensorTypeAndShapeInfo();
    const std::vector<int64_t> oshape = info.GetShape();
    const ONNXTensorElementDataType et = info.GetElementType();
    cv::Mat class_map;

    if (oshape.size() == 4 && oshape[1] > 1 &&
        et == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        const int C = static_cast<int>(oshape[1]);
        const int h = static_cast<int>(oshape[2]);
        const int w = static_cast<int>(oshape[3]);
        class_map = ArgmaxLogitsToClassMap(out.GetTensorData<float>(), C, h, w);
    } else if (oshape.size() == 4 && oshape[1] == 1 &&
               et == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        const int h = static_cast<int>(oshape[2]);
        const int w = static_cast<int>(oshape[3]);
        return ProbMapToKeep(out.GetTensorData<float>(), h, w, mask_threshold,
                             keep_out);
    } else if ((oshape.size() == 3 || (oshape.size() == 4 && oshape[1] == 1)) &&
               (et == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64 ||
                et == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32)) {
        class_map = LabelsToClassMap(out, oshape, et);
    } else {
        std::cerr << "[semantic] unsupported ONNX output shape/type\n";
        return false;
    }

    *keep_out = KeepFromClassMap(class_map, dyn);
    return true;
}
#endif  // ARGUS_HAS_ONNXRUNTIME

}  // namespace

struct SemanticDynamicFilter::OnnxSession {
#ifdef ARGUS_HAS_ONNXRUNTIME
    Ort::Env env{ORT_LOGGING_LEVEL_ERROR, "argus_semantic"};
    Ort::SessionOptions options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;
    std::string input_name;
    std::string output_name;
    std::vector<int64_t> input_shape;
#endif
};

SemanticDynamicFilter::SemanticDynamicFilter(SemanticDynamicFilterConfig config)
    : config_(std::move(config)) {
    if (config_.mean.size() != 3) {
        config_.mean = {0.485f, 0.456f, 0.406f};
    }
    if (config_.std.size() != 3) {
        config_.std = {0.229f, 0.224f, 0.225f};
    }
    if (config_.backend == SemanticBackend::kMaskFile) {
        if (!config_.mask_path.empty()) {
            LoadMaskFile(config_.mask_path);
        }
    } else {
        model_loaded_ = InitOnnx();
    }
}

SemanticDynamicFilter::~SemanticDynamicFilter() = default;

void SemanticDynamicFilter::SetLidarCameraExtrinsic(
    const Eigen::Matrix3d& R_cl, const Eigen::Vector3d& t_cl) {
    config_.R_cl = R_cl;
    config_.t_cl = t_cl;
}

void SemanticDynamicFilter::SetCameraIntrinsics(double fx, double fy, double cx,
                                                double cy, int width,
                                                int height) {
    config_.fx = fx;
    config_.fy = fy;
    config_.cx = cx;
    config_.cy = cy;
    if (width > 0) {
        config_.image_width = width;
    }
    if (height > 0) {
        config_.image_height = height;
    }
}

void SemanticDynamicFilter::SetKeepMask(const cv::Mat& keep_mask) {
    if (keep_mask.empty()) {
        keep_mask_.release();
        return;
    }
    if (keep_mask.type() == CV_8UC1) {
        keep_mask_ = keep_mask.clone();
    } else {
        keep_mask.convertTo(keep_mask_, CV_8UC1);
    }
}

bool SemanticDynamicFilter::LoadMaskFile(const std::string& path) {
    const std::string p = path.empty() ? config_.mask_path : path;
    if (p.empty()) {
        return false;
    }
    cv::Mat m = cv::imread(p, cv::IMREAD_GRAYSCALE);
    if (m.empty()) {
        std::cerr << "[semantic] failed to load mask: " << p << std::endl;
        return false;
    }
    // Convention: file 0 = dynamic (reject), >0 = keep.
    cv::Mat keep;
    cv::threshold(m, keep, 0, 1, cv::THRESH_BINARY);
    keep_mask_ = keep;
    config_.image_width = keep.cols;
    config_.image_height = keep.rows;
    return true;
}

bool SemanticDynamicFilter::InitOnnx() {
#ifndef ARGUS_HAS_ONNXRUNTIME
    if (!config_.model_path.empty()) {
        std::cerr << "[semantic] ONNX Runtime not linked; pass-through "
                     "(set ARGUS_USE_ONNXRUNTIME / install onnxruntime)\n";
    }
    return false;
#else
    if (config_.model_path.empty()) {
        return false;
    }
    try {
        onnx_ = std::make_unique<OnnxSession>();
        onnx_->options.SetIntraOpNumThreads(1);
        onnx_->options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_BASIC);
        onnx_->session = std::make_unique<Ort::Session>(
            onnx_->env, config_.model_path.c_str(), onnx_->options);
        Ort::AllocatedStringPtr in_name =
            onnx_->session->GetInputNameAllocated(0, onnx_->allocator);
        Ort::AllocatedStringPtr out_name =
            onnx_->session->GetOutputNameAllocated(0, onnx_->allocator);
        onnx_->input_name = in_name.get();
        onnx_->output_name = out_name.get();
        auto type_info = onnx_->session->GetInputTypeInfo(0);
        auto shape = type_info.GetTensorTypeAndShapeInfo().GetShape();
        onnx_->input_shape = shape;
        if (shape.size() == 4) {
            if (shape[2] > 0) {
                config_.input_height = static_cast<int>(shape[2]);
            }
            if (shape[3] > 0) {
                config_.input_width = static_cast<int>(shape[3]);
            }
        }
        return true;
    } catch (const Ort::Exception& e) {
        std::cerr << "[semantic] ONNX load failed: " << e.what() << std::endl;
        onnx_.reset();
        return false;
    }
#endif
}

bool SemanticDynamicFilter::RunOnnx(const cv::Mat& bgr, cv::Mat* keep_out) {
#ifndef ARGUS_HAS_ONNXRUNTIME
    (void)bgr;
    (void)keep_out;
    return false;
#else
    if (!model_loaded_ || !onnx_ || !onnx_->session || keep_out == nullptr) {
        return false;
    }
    const int H = config_.input_height;
    const int W = config_.input_width;
    std::vector<float> input =
        MakeNchwInput(bgr, H, W, config_.mean, config_.std);

    std::array<int64_t, 4> shape{1, 3, H, W};
    Ort::MemoryInfo mem =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem, input.data(), input.size(), shape.data(), shape.size());

    const char* in_names[] = {onnx_->input_name.c_str()};
    const char* out_names[] = {onnx_->output_name.c_str()};
    auto outputs = onnx_->session->Run(Ort::RunOptions{nullptr}, in_names,
                                       &input_tensor, 1, out_names, 1);
    if (outputs.empty()) {
        return false;
    }

    std::unordered_set<int> dyn(config_.dynamic_class_ids.begin(),
                                config_.dynamic_class_ids.end());
    return DecodeOnnxKeepMask(outputs[0], config_.mask_threshold, dyn,
                              keep_out);
#endif
}

Mask SemanticDynamicFilter::FilterVisual(const cv::Mat& image,
                                         const StatesGroup& /*state*/) {
    Mask m;
    if (image.empty()) {
        return m;
    }

    if (config_.backend == SemanticBackend::kMaskFile) {
        if (keep_mask_.empty() && !config_.mask_path.empty()) {
            LoadMaskFile(config_.mask_path);
        }
    } else if (model_loaded_) {
        cv::Mat keep;
        if (RunOnnx(image, &keep)) {
            // Resize mask to original image size for projection.
            if (keep.cols != image.cols || keep.rows != image.rows) {
                cv::Mat resized;
                cv::resize(keep, resized, image.size(), 0, 0,
                           cv::INTER_NEAREST);
                keep_mask_ = resized;
            } else {
                keep_mask_ = keep;
            }
            config_.image_width = image.cols;
            config_.image_height = image.rows;
        }
    }

    if (!keep_mask_.empty()) {
        m.keep.assign(
            static_cast<std::size_t>(keep_mask_.rows * keep_mask_.cols), 1);
        for (int r = 0; r < keep_mask_.rows; ++r) {
            const uint8_t* row = keep_mask_.ptr<uint8_t>(r);
            for (int c = 0; c < keep_mask_.cols; ++c) {
                m.keep[static_cast<std::size_t>(r * keep_mask_.cols + c)] =
                    row[c] ? 1 : 0;
            }
        }
    } else {
        m.keep.assign(static_cast<std::size_t>(image.rows * image.cols), 1);
    }
    return m;
}

Mask SemanticDynamicFilter::ProjectCloud(
    const sensor::PointCloudXYZI& cloud) const {
    Mask m;
    m.keep.assign(cloud.size(), 1);
    if (keep_mask_.empty()) {
        return m;
    }
    const int mw = keep_mask_.cols;
    const int mh = keep_mask_.rows;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud[i];
        const Eigen::Vector3d pl(p.x, p.y, p.z);
        const Eigen::Vector3d pc = config_.R_cl * pl + config_.t_cl;
        if (pc.z() <= 1e-3) {
            continue;
        }
        const double u = config_.fx * (pc.x() / pc.z()) + config_.cx;
        const double v = config_.fy * (pc.y() / pc.z()) + config_.cy;
        const int iu = static_cast<int>(std::lround(u));
        const int iv = static_cast<int>(std::lround(v));
        if (iu < 0 || iv < 0 || iu >= mw || iv >= mh) {
            continue;
        }
        if (keep_mask_.at<uint8_t>(iv, iu) == 0) {
            m.keep[i] = 0;
        }
    }
    return m;
}

Mask SemanticDynamicFilter::FilterLidar(const sensor::PointCloudXYZI& cloud,
                                        const StatesGroup& /*state*/) {
    if (keep_mask_.empty()) {
        Mask m;
        m.keep.assign(cloud.size(), 1);
        return m;
    }
    return ProjectCloud(cloud);
}

}  // namespace frontend
}  // namespace argus
