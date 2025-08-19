/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/localization/utils/image_loader.hpp"

#include <cstdint>
#include <filesystem>
#include <algorithm>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace localization {
namespace utils {


commsgs::sensor_msgs::Image FromCV(const cv::Mat& mat)
{
    commsgs::sensor_msgs::Image image;

    // 设置图像尺寸
    image.height = static_cast<uint32_t>(mat.rows);
    image.width = static_cast<uint32_t>(mat.cols);
    
    // 设置步长（每行的字节数）
    image.step = static_cast<uint32_t>(image.step);
    
    // 设置字节序（假设为小端序）
    image.is_bigendian = 0;
    
    // 设置编码格式
    switch (mat.type()) {
        case CV_8UC1:
            image.encoding = "mono8";
            break;
        case CV_8UC3:
            image.encoding = "bgr8"; // OpenCV 默认使用 BGR 顺序
            break;
        case CV_8UC4:
            image.encoding = "bgra8";
            break;
        case CV_16UC1:
            image.encoding = "mono16";
            break;
        case CV_32FC1:
            image.encoding = "32FC1";
            break;
        case CV_32FC3:
            image.encoding = "32FC3";
            break;
        default:
            image.encoding = "unknown";
            break;
    }
     
     // 复制数据
     size_t data_size = mat.total() * mat.elemSize();
     image.data.resize((data_size + sizeof(uint32_t) - 1) / sizeof(uint32_t));
     
     // 将字节数据复制到 uint32_t 向量中
     const uint8_t* mat_data = mat.ptr<uint8_t>();
     for (size_t i = 0; i < data_size; i += sizeof(uint32_t)) {
         uint32_t value = 0;
         size_t bytes_to_copy = std::min(sizeof(uint32_t), data_size - i);
         
         // 将字节复制到 uint32_t 中
         std::memcpy(&value, mat_data + i, bytes_to_copy);
         image.data[i / sizeof(uint32_t)] = value;
     }
     
    return image;
}

bool ReadImage(const std::string& directory_path, std::vector<cv::Mat>& images)
{
    // 检查目录是否存在
    if (!std::filesystem::exists(directory_path)) {
        LOG(ERROR) << "Directory does not exist: " << directory_path;
        return false;
    }

    // 检查是否为目录
    if (!std::filesystem::is_directory(directory_path)) {
        LOG(ERROR) << "Path is not a directory: " << directory_path;
        return false;
    }

    // 支持的图片格式扩展名
    std::vector<std::string> supported_extensions = {
        ".jpg", ".jpeg", ".png", ".bmp"
    };
    
    // 遍历目录中的所有文件
    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (std::filesystem::is_regular_file(entry)) {
            std::string extension = entry.path().extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
            if (std::find(supported_extensions.begin(), supported_extensions.end(), extension) != supported_extensions.end()) {
        
                cv::Mat image = cv::imread(entry.path().string());
                if (!image.empty()) {
                    images.push_back(image);
                } else {
                    LOG(ERROR) << "Failed to read image: " << entry.path().string();
                }
            }
        }
    }

    return true;
}

bool ReadImage(const std::string& directory_path, std::vector<commsgs::sensor_msgs::Image>& images)
{
    std::vector<cv::Mat> cv_images;
    if (!ReadImage(directory_path, cv_images)) {
        return false;
    }

    for (auto const& image : cv_images) {  
        images.push_back(FromCV(image));
    }
    return true;
}

}  // namespace utils
}  // namespace localization
}  // namespace autonomy