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

#include "autonomy/tools/god_viewer/channel/channel_image_handler.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/localization/utils/image_loader.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

std::vector<cv::Mat> images;

ImageHandler::ImageHandler(ServerHander::SharedPtr options, const std::string& topic)
    : topic_{topic}
{
    LOG(INFO) << "Init handler topic: " << topic_;
    channel_ = std::make_unique<foxglove::schemas::RawImageChannel>(
        foxglove::schemas::RawImageChannel::create(topic_).value());

    std::string image_path = "/home/quandy/Downloads/recording_2020-10-08_10-19-46_stereo_images_undistorted/recording_2020-10-08_10-19-46/undistorted_images/cam0/";

    localization::utils::ReadImage(image_path, images);
}

bool ImageHandler::SendTest()
{
  
    if (images.empty()) {
        return false;
    }

    static int count = 0;

    Send(images[count++]);

    if (count >= images.size()) {
        count = 0;
    }

    return true;
}

bool ImageHandler::Send(const commsgs::sensor_msgs::Image& msgs)
{
    auto data = FromCommsgs(msgs);
    channel_->log(data);
    return true;
}

bool ImageHandler::Send(const cv::Mat& mat)
{

    foxglove::schemas::RawImage rawImage;
    
    // 设置帧ID
    rawImage.frame_id = "map";
    
    // 设置图像尺寸
    rawImage.width = static_cast<uint32_t>(mat.cols);
    rawImage.height = static_cast<uint32_t>(mat.rows);
    
    // 设置步长（每行的字节数）
    rawImage.step = static_cast<uint32_t>(mat.step);
    
    // 设置编码格式
    switch (mat.type()) {
        case CV_8UC1:
            rawImage.encoding = "mono8";
            break;
        case CV_8UC3:
            rawImage.encoding = "bgr8"; // OpenCV 默认使用 BGR 顺序
            break;
        case CV_8UC4:
            rawImage.encoding = "bgra8";
            break;
        case CV_16UC1:
            rawImage.encoding = "mono16";
            break;
        case CV_16UC3:
            rawImage.encoding = "bgr16";
            break;
        case CV_32FC1:
            rawImage.encoding = "32fc1";
            break;
        case CV_32FC3:
            rawImage.encoding = "32fc3";
            break;
        default:
            rawImage.encoding = "unknown";
            break;
    }
    
    // 复制数据
    size_t data_size = mat.total() * mat.elemSize();
    rawImage.data.resize(data_size);
    
    // 检查矩阵是否连续（没有填充）
    if (mat.isContinuous()) {
        // 直接复制整个数据块
        std::memcpy(rawImage.data.data(), mat.data, data_size);
    } else {
        // 逐行复制数据（处理有填充的情况）
        size_t offset = 0;
        for (int i = 0; i < mat.rows; ++i) {
            const uchar* row_data = mat.ptr<uchar>(i);
            std::memcpy(rawImage.data.data() + offset, row_data, mat.step);
            offset += mat.step;
        }
    }


    channel_->log(rawImage);
    return true;
}

foxglove::schemas::RawImage ImageHandler::FromCommsgs(const commsgs::sensor_msgs::Image& msgs)
{

    //     struct RawImage {

    
    //     /// @brief Frame of reference for the image. The origin of the frame is the optical center of the
    //     /// camera. +x points to the right in the image, +y points down, and +z points into the plane of
    //     /// the image.
    //     std::string frame_id;
    
    //     /// @brief Image width in pixels
    //     uint32_t width;
    
    //     /// @brief Image height in pixels
    //     uint32_t height;
    
    //     /// @brief Encoding of the raw image data. See the `data` field description for supported values.
    //     std::string encoding;
    
    //     /// @brief Byte length of a single row. This is usually some multiple of `width` depending on the
    //     /// encoding, but can be greater to incorporate padding.
    //     uint32_t step;
    
    //     std::vector<std::byte> data;
    // };


    // struct Image 
    // {

    //     // image height, that is, number of rows
    //     uint32 height;       
        
    //     // image width, that is, number of columns
    //     uint32 width;            

    //     // The legal values for encoding are in file src/image_encodings.cpp
    //     // If you want to standardize a new std::string format, join
    //     // ros-users@lists.ros.org and send an email proposing a new encoding.
    //     std::string encoding;    // Encoding of pixels -- channel meaning, ordering, size
    //                             // taken from the list of std::strings in include/sensor_msgs/image_encodings.hpp


    //     // is this data bigendian?
    //     uint32 is_bigendian;

    //     // Full row length in bytes
    //     uint32 step;

    //     // actual matrix data, size is (step * rows)
    //     std::vector<uint32> data;
    // };


  
    foxglove::schemas::RawImage image;
    image.frame_id = "map";
    image.width = msgs.width;
    image.height = msgs.height;
    // image.encoding = msgs.encoding;

    LOG(INFO) << "msgs.encoding: " << msgs.encoding;
    image.encoding = "mono8";
    image.step = msgs.step;

    // 转换数据：从 std::vector<uint32_t> 到 std::vector<std::byte>
    size_t byte_count = image.width * image.height;
    image.data.resize(byte_count);
    
    // 将 uint32_t 向量中的数据复制回字节形式
    std::byte* raw_data = image.data.data();
    for (size_t i = 0; i < byte_count; i += sizeof(uint32_t)) {
        uint32_t value = msgs.data[i / sizeof(uint32_t)];
        size_t bytes_to_copy = std::min(sizeof(uint32_t), byte_count - i);
        
        // 将 uint32_t 复制回字节
        std::memcpy(raw_data + i, &value, bytes_to_copy);
    }

    return image;
}

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy