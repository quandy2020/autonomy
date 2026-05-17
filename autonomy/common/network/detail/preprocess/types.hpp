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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_TYPES_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_TYPES_HPP_

#include "autonomy/common/network/tensor.hpp"

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file types.hpp
 * @brief Preprocess configuration types and inference sample containers
 */

/**
 * @brief Raw observation bound to model inputs before preprocessing
 */
struct Sample {
    std::optional<cv::Mat> image_bgr;  //!< @brief Optional BGR image (8-bit)
    std::vector<float> vector_features;  //!< @brief Optional 1-D feature vector for vector inputs
    TensorMap named_tensors;  //!< @brief Pre-built tensors keyed by ONNX input name
};

/**
 * @brief How a BGR image is resized to the model spatial size
 */
enum class ResizePolicy {
    kLetterbox,   //!< @brief Preserve aspect ratio with padding
    kStretch,     //!< @brief Resize to exact HxW without preserving aspect ratio
    kCenterCrop,  //!< @brief Scale and center-crop to target size
    kUpperBound,  //!< @brief Scale longest side to bound_resize_target, then fit to model HxW
};

/**
 * @brief Memory layout of the preprocessed image blob
 */
enum class LayoutPolicy {
    kNCHW,  //!< @brief Channels-first (batch, C, H, W)
    kNHWC,  //!< @brief Channels-last (batch, H, W, C)
    kAuto,  //!< @brief Infer from model input tensor shape
};

/**
 * @brief Normalization applied after resize and before inference
 */
enum class NormalizePolicy {
    kZeroOne,       //!< @brief Scale pixel values to [0, 1]
    kImageNet,      //!< @brief ImageNet mean/std (with BGR→RGB swap when enabled)
    kMinusOneToOne, //!< @brief Map to [-1, 1]
    kCustom,        //!< @brief Use custom_normalize mean and std
};

/**
 * @brief Per-channel mean and std in RGB order (after optional BGR→RGB swap)
 */
struct NormalizeParams {
    float mean[3] = {0.f, 0.f, 0.f};  //!< @brief Mean for R, G, B
    float std[3] = {1.f, 1.f, 1.f};   //!< @brief Std for R, G, B
};

/**
 * @brief Geometry metadata for mapping network coordinates back to the source image
 */
struct TransformMeta {
    ResizePolicy resize_policy = ResizePolicy::kLetterbox;  //!< @brief Resize mode used in preprocess
    double scale_gain = 1.0;        //!< @brief Uniform scale from source to letterbox/crop space
    int padding_left = 0;           //!< @brief Left padding in preprocessed image (pixels)
    int padding_top = 0;            //!< @brief Top padding in preprocessed image (pixels)
    int crop_offset_x = 0;          //!< @brief Horizontal crop offset after scale (pixels)
    int crop_offset_y = 0;          //!< @brief Vertical crop offset after scale (pixels)
    int input_height = 0;           //!< @brief Height fed to the model
    int input_width = 0;            //!< @brief Width fed to the model
    int source_height = 0;          //!< @brief Original image height
    int source_width = 0;           //!< @brief Original image width
};

/**
 * @brief Intermediate result of letterbox resize (internal/debug use)
 */
struct LetterboxOutput {
    cv::Mat image;           //!< @brief Letterboxed BGR image
    double scale_ratio = 1.0;  //!< @brief Scale factor applied to source
    int pad_width = 0;       //!< @brief Total horizontal padding
    int pad_height = 0;      //!< @brief Total vertical padding
    TransformMeta meta;      //!< @brief Geometry for inverse mapping
};

/**
 * @brief Unified configuration for @ref Preprocess and @ref RunPipeline
 */
struct PreprocessOptions {
    ResizePolicy resize = ResizePolicy::kLetterbox;       //!< @brief Spatial resize policy
    NormalizePolicy normalize = NormalizePolicy::kZeroOne;  //!< @brief Pixel normalization policy
    NormalizeParams custom_normalize{};  //!< @brief Mean/std when normalize is kCustom
    LayoutPolicy layout = LayoutPolicy::kAuto;  //!< @brief Output tensor layout
    bool swap_red_blue = true;  //!< @brief If true, swap BGR to RGB before mean subtraction
    int pad_value = 114;        //!< @brief Letterbox pad color (BGR gray)
    int default_height = 640;   //!< @brief Fallback height when model shape is dynamic
    int default_width = 640;    //!< @brief Fallback width when model shape is dynamic
    int bound_resize_target = 0;  //!< @brief Longest-side target for kUpperBound (0 = use model H/W)
    int align_multiple = 0;     //!< @brief If >0, round H/W up to this multiple (e.g. patch 14)
};

/**
 * @brief Parsed spatial size and layout from a 4-D or 5-D image input descriptor
 */
struct ImageInputShape {
    int height = 0;              //!< @brief Model input height
    int width = 0;               //!< @brief Model input width
    int channel_count = 3;       //!< @brief Number of color channels
    LayoutPolicy layout = LayoutPolicy::kNCHW;  //!< @brief Inferred layout
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_TYPES_HPP_
