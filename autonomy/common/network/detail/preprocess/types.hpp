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

#include "autonomy/common/network/common/tensor.hpp"

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file types.hpp
 * @brief Preprocess configuration, sample containers, and geometry metadata
 *
 * Shared by @ref Preprocess and @ref RunPipeline. Policy enums and @ref PreprocessOptions
 * are part of the public API.
 */

/**
 * @brief Raw observation bound to model inputs before preprocessing
 */
struct Sample {
    std::optional<cv::Mat> image_bgr;  //!< Optional BGR image (8-bit)
    std::vector<float> vector_features;  //!< Optional 1-D features for vector inputs
    TensorMap named_tensors;  //!< Pre-built tensors keyed by ONNX input name
};

/**
 * @brief How a BGR image is resized to the model spatial size
 */
enum class ResizePolicy {
    kLetterbox,   //!< Preserve aspect ratio with padding
    kStretch,     //!< Resize to exact H×W without preserving aspect ratio
    kCenterCrop,  //!< Scale and center-crop to target size
    kUpperBound,  //!< Scale longest side to bound_resize_target, then fit to model H×W
};

/**
 * @brief Memory layout of the preprocessed image blob
 */
enum class LayoutPolicy {
    kNCHW,  //!< Channels-first (batch, C, H, W)
    kNHWC,  //!< Channels-last (batch, H, W, C)
    kAuto,  //!< Infer from model input tensor shape
};

/**
 * @brief Normalization applied after resize and before inference
 */
enum class NormalizePolicy {
    kZeroOne,       //!< Scale pixel values to [0, 1]
    kImageNet,      //!< ImageNet mean/std (with BGR→RGB swap when enabled)
    kMinusOneToOne, //!< Map to [-1, 1]
    kCustom,        //!< Use custom_normalize mean and std
};

/**
 * @brief Per-channel mean and std in RGB order (after optional BGR→RGB swap)
 */
struct NormalizeParams {
    float mean[3] = {0.f, 0.f, 0.f};  //!< Mean for R, G, B
    float std[3] = {1.f, 1.f, 1.f};   //!< Std for R, G, B
};

/**
 * @brief Geometry metadata for mapping network coordinates back to the source image
 */
struct TransformMeta {
    ResizePolicy resize_policy = ResizePolicy::kLetterbox;  //!< Resize mode used in preprocess
    double scale_gain = 1.0;        //!< Uniform scale from source to letterbox/crop space
    int padding_left = 0;           //!< Left padding in preprocessed image (pixels)
    int padding_top = 0;            //!< Top padding in preprocessed image (pixels)
    int crop_offset_x = 0;          //!< Horizontal crop offset after scale (pixels)
    int crop_offset_y = 0;          //!< Vertical crop offset after scale (pixels)
    int input_height = 0;           //!< Height fed to the model
    int input_width = 0;            //!< Width fed to the model
    int source_height = 0;          //!< Original image height
    int source_width = 0;           //!< Original image width
};

/**
 * @brief Intermediate result of letterbox resize (internal/debug use)
 */
struct LetterboxOutput {
    cv::Mat image;           //!< Letterboxed BGR image
    double scale_ratio = 1.0;  //!< Scale factor applied to source
    int pad_width = 0;       //!< Total horizontal padding
    int pad_height = 0;      //!< Total vertical padding
    TransformMeta meta;      //!< Geometry for inverse mapping
};

/**
 * @brief Unified configuration for @ref Preprocess and @ref RunPipeline
 */
struct PreprocessOptions {
    ResizePolicy resize = ResizePolicy::kLetterbox;       //!< Spatial resize policy
    NormalizePolicy normalize = NormalizePolicy::kZeroOne;  //!< Pixel normalization policy
    NormalizeParams custom_normalize{};  //!< Mean/std when normalize is kCustom
    LayoutPolicy layout = LayoutPolicy::kAuto;  //!< Output tensor layout
    bool swap_red_blue = true;  //!< If true, swap BGR to RGB before mean subtraction
    int pad_value = 114;        //!< Letterbox pad color (BGR gray)
    int default_height = 640;   //!< Fallback height when model shape is dynamic
    int default_width = 640;    //!< Fallback width when model shape is dynamic
    int bound_resize_target = 0;  //!< Longest-side target for kUpperBound (0 = use model H/W)
    int align_multiple = 0;     //!< If >0, round H/W up to this multiple (e.g. patch 14)
};

/**
 * @brief Parsed spatial size and layout from a 4-D or 5-D image input descriptor
 */
struct ImageInputShape {
    int height = 0;              //!< Model input height
    int width = 0;               //!< Model input width
    int channel_count = 3;       //!< Number of color channels
    LayoutPolicy layout = LayoutPolicy::kNCHW;  //!< Inferred layout
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_TYPES_HPP_
