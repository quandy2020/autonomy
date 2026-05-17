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

#include "autonomy/common/network/detail/postprocess/map.hpp"

#include "autonomy/common/network/detail/postprocess/internal/error.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace autonomy {
namespace common {
namespace network {

namespace {
using postprocess_internal::SetErrorMessage;

bool ContainsNameKeyword(const std::string& name, const std::string& keyword) {
    if (keyword.empty()) {
        return false;
    }
    std::string lower_name = name;
    std::string lower_keyword = keyword;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(),
                 [](unsigned char character) {
                     return static_cast<char>(std::tolower(character));
                 });
    std::transform(lower_keyword.begin(), lower_keyword.end(), lower_keyword.begin(),
                 [](unsigned char character) {
                     return static_cast<char>(std::tolower(character));
                 });
    return lower_name.find(lower_keyword) != std::string::npos;
}

int64_t SpatialElementCount(const ModelTensorInfo& tensor_info) {
    const std::vector<int64_t>& dimensions = tensor_info.shape.Dims();
    if (dimensions.size() < 2) {
        return 0;
    }
    int64_t count = 1;
    const size_t start_index = dimensions.size() - 2;
    for (size_t index = start_index; index < dimensions.size(); ++index) {
        if (dimensions[index] > 0) {
            count *= dimensions[index];
        }
    }
    return count;
}

bool InferMapShape(const ModelTensorInfo& info, size_t element_count, int* height, int* width,
                   size_t* offset) {
    const std::vector<int64_t>& dimensions = info.shape.Dims();
    if (dimensions.size() < 2) {
        return false;
    }
    const int64_t map_height = dimensions[dimensions.size() - 2];
    const int64_t map_width = dimensions[dimensions.size() - 1];
    if (map_height <= 0 || map_width <= 0) {
        return false;
    }
    int64_t leading = 1;
    for (size_t index = 0; index + 2 < dimensions.size(); ++index) {
        if (dimensions[index] > 0) {
            leading *= dimensions[index];
        }
    }
    const int64_t spatial = map_height * map_width;
    if (spatial <= 0 || static_cast<size_t>(leading * spatial) > element_count) {
        return false;
    }
    *height = static_cast<int>(map_height);
    *width = static_cast<int>(map_width);
    *offset = 0;
    return true;
}

}  // namespace

bool Find(
    const std::unordered_map<std::string, std::vector<float>>& outputs,
    const std::vector<ModelTensorInfo>& output_infos, std::string* output_name,
    const std::vector<float>** output_data, const std::string& name_keyword,
    std::string* error) {
    if (output_name == nullptr || output_data == nullptr) {
        SetErrorMessage(error, "output_name or output_data is null.");
        return false;
    }
    if (outputs.empty()) {
        SetErrorMessage(error, "model produced no outputs.");
        return false;
    }

    if (!name_keyword.empty()) {
        const auto exact = outputs.find(name_keyword);
        if (exact != outputs.end()) {
            *output_name = name_keyword;
            *output_data = &exact->second;
            return true;
        }
    }

    const std::vector<float>* best_data = nullptr;
    std::string best_name;
    int64_t best_spatial_count = -1;

    for (const ModelTensorInfo& tensor_info : output_infos) {
        const auto found = outputs.find(tensor_info.name);
        if (found == outputs.end()) {
            continue;
        }
        if (ContainsNameKeyword(tensor_info.name, name_keyword)) {
            *output_name = tensor_info.name;
            *output_data = &found->second;
            return true;
        }
        const int64_t spatial_count = SpatialElementCount(tensor_info);
        if (spatial_count > best_spatial_count) {
            best_spatial_count = spatial_count;
            best_data = &found->second;
            best_name = tensor_info.name;
        }
    }

    if (best_data != nullptr) {
        *output_name = best_name;
        *output_data = best_data;
        return true;
    }

    const auto first = outputs.begin();
    *output_name = first->first;
    *output_data = &first->second;
    return true;
}

bool ToMat(const std::vector<float>& output, const ModelTensorInfo& info,
           const TransformMeta& meta, cv::Mat* mat, std::string* error) {
    if (mat == nullptr) {
        SetErrorMessage(error, "mat output is null.");
        return false;
    }
    if (output.empty()) {
        SetErrorMessage(error, "empty map output.");
        return false;
    }

    int height = meta.input_height > 0 ? meta.input_height : 0;
    int width = meta.input_width > 0 ? meta.input_width : 0;
    size_t offset = 0;
    if (!InferMapShape(info, output.size(), &height, &width, &offset)) {
        if (height <= 0 || width <= 0) {
            const int side = static_cast<int>(std::sqrt(static_cast<double>(output.size())));
            height = side;
            width = side;
        }
    }
    if (height <= 0 || width <= 0 ||
        offset + static_cast<size_t>(height * width) > output.size()) {
        SetErrorMessage(error, "map size mismatch.");
        return false;
    }

    cv::Mat small_map(height, width, CV_32FC1);
    for (int row = 0; row < height; ++row) {
        float* row_ptr = small_map.ptr<float>(row);
        for (int col = 0; col < width; ++col) {
            row_ptr[col] = output[offset + static_cast<size_t>(row * width + col)];
        }
    }

    const int output_height = meta.source_height > 0 ? meta.source_height : height;
    const int output_width = meta.source_width > 0 ? meta.source_width : width;
    cv::resize(small_map, *mat, cv::Size(output_width, output_height), 0, 0, cv::INTER_LINEAR);
    return true;
}

bool Colorize(const cv::Mat& float_map, cv::Mat* bgr, int colormap) {
    if (bgr == nullptr || float_map.empty()) {
        return false;
    }
    cv::Mat normalized_map;
    double min_value = 0.0;
    double max_value = 0.0;
    cv::minMaxLoc(float_map, &min_value, &max_value);
    if (max_value - min_value < 1e-6) {
        max_value = min_value + 1.0;
    }
    float_map.convertTo(normalized_map, CV_8U, 255.0 / (max_value - min_value),
                        -min_value * 255.0 / (max_value - min_value));
    const int cv_colormap = colormap >= 0 ? colormap : cv::COLORMAP_INFERNO;
    cv::applyColorMap(normalized_map, *bgr, cv_colormap);
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
