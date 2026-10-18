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

#include "argus/sensor/image_data.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace argus {
namespace sensor {
namespace {

double StampToSec(const ::automsgs::msgs::builtin_interfaces::Time& stamp) {
    return static_cast<double>(stamp.sec()) +
           1e-9 * static_cast<double>(stamp.nanosec());
}

void SetStamp(::automsgs::msgs::builtin_interfaces::Time* stamp,
              double timestamp_sec) {
    if (stamp == nullptr) {
        return;
    }
    auto sec = static_cast<int32_t>(timestamp_sec);
    auto nanosec = static_cast<uint32_t>(
        std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
    if (nanosec >= 1000000000u) {
        stamp->set_sec(sec + 1);
        stamp->set_nanosec(nanosec - 1000000000u);
    } else {
        stamp->set_sec(sec);
        stamp->set_nanosec(nanosec);
    }
}

void FillHeader(::automsgs::msgs::std_msgs::Header* header,
                double timestamp_sec, const std::string& frame_id) {
    SetStamp(header->mutable_stamp(), timestamp_sec);
    header->set_frame_id(frame_id);
}

/** @brief Normalize decoded Mat to mono8 or BGR; set encoding. */
bool NormalizeImage(cv::Mat* image, const std::string& src_encoding,
                    std::string* out_encoding) {
    if (image == nullptr || image->empty() || out_encoding == nullptr) {
        return false;
    }
    if (src_encoding == "mono8" || src_encoding == "8UC1") {
        *out_encoding = "mono8";
        return true;
    }
    if (src_encoding == "bgr8" || src_encoding == "8UC3") {
        *out_encoding = "bgr8";
        return true;
    }
    if (src_encoding == "rgb8") {
        cv::cvtColor(*image, *image, cv::COLOR_RGB2BGR);
        *out_encoding = "bgr8";
        return true;
    }
    if (src_encoding == "bgra8" || src_encoding == "8UC4") {
        cv::cvtColor(*image, *image, cv::COLOR_BGRA2BGR);
        *out_encoding = "bgr8";
        return true;
    }
    if (src_encoding == "rgba8") {
        cv::cvtColor(*image, *image, cv::COLOR_RGBA2BGR);
        *out_encoding = "bgr8";
        return true;
    }
    if (src_encoding == "mono16" || src_encoding == "16UC1") {
        image->convertTo(*image, CV_8UC1, 1.0 / 256.0);
        *out_encoding = "mono8";
        return true;
    }
    // Unknown encoding: keep Mat as-is if already 1 or 3 channels.
    if (image->type() == CV_8UC1) {
        *out_encoding = "mono8";
        return true;
    }
    if (image->type() == CV_8UC3) {
        *out_encoding = "bgr8";
        return true;
    }
    image->release();
    return false;
}

::automsgs::msgs::sensor_msgs::Image EncodeMat(const cv::Mat& image,
                                               double timestamp_sec,
                                               const std::string& frame_id,
                                               const std::string& encoding) {
    ::automsgs::msgs::sensor_msgs::Image out;
    FillHeader(out.mutable_header(), timestamp_sec, frame_id);
    if (image.empty()) {
        return out;
    }
    out.set_height(static_cast<uint32_t>(image.rows));
    out.set_width(static_cast<uint32_t>(image.cols));
    out.set_encoding(encoding);
    out.set_is_bigendian(false);
    const size_t step = static_cast<size_t>(image.cols) * image.elemSize();
    out.set_step(static_cast<uint32_t>(step));
    if (image.isContinuous()) {
        out.set_data(image.data, image.rows * step);
    } else {
        std::string bytes;
        bytes.reserve(static_cast<size_t>(image.rows) * step);
        for (int r = 0; r < image.rows; ++r) {
            bytes.append(reinterpret_cast<const char*>(image.ptr(r)), step);
        }
        out.set_data(std::move(bytes));
    }
    return out;
}

}  // namespace

ImageData ImageData::Clone() const {
    ImageData out;
    out.timestamp = timestamp;
    out.frame_id = frame_id;
    out.encoding = encoding;
    out.image = image.clone();
    return out;
}

cv::Mat ImageData::ToGray() const {
    if (image.empty()) {
        return {};
    }
    if (image.channels() == 1) {
        return image.clone();
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

ImageData FromProto(const ::automsgs::msgs::sensor_msgs::Image& msg) {
    ImageData out;
    if (msg.has_header()) {
        if (msg.header().has_stamp()) {
            out.timestamp = StampToSec(msg.header().stamp());
        }
        out.frame_id = msg.header().frame_id();
    }
    if (msg.height() == 0 || msg.width() == 0 || msg.data().empty()) {
        return out;
    }

    int type = -1;
    if (msg.encoding() == "mono8" || msg.encoding() == "8UC1") {
        type = CV_8UC1;
    } else if (msg.encoding() == "mono16" || msg.encoding() == "16UC1") {
        type = CV_16UC1;
    } else if (msg.encoding() == "bgr8" || msg.encoding() == "rgb8" ||
               msg.encoding() == "8UC3") {
        type = CV_8UC3;
    } else if (msg.encoding() == "bgra8" || msg.encoding() == "rgba8" ||
               msg.encoding() == "8UC4") {
        type = CV_8UC4;
    }
    if (type < 0) {
        return out;
    }

    auto* ptr = reinterpret_cast<void*>(const_cast<char*>(msg.data().data()));
    const int rows = static_cast<int>(msg.height());
    const int cols = static_cast<int>(msg.width());
    if (msg.step() == 0) {
        out.image = cv::Mat(rows, cols, type, ptr).clone();
    } else {
        out.image =
            cv::Mat(rows, cols, type, ptr, static_cast<size_t>(msg.step()))
                .clone();
    }
    if (!NormalizeImage(&out.image, msg.encoding(), &out.encoding)) {
        out.image.release();
    }
    return out;
}

ImageData FromProto(const ::automsgs::msgs::sensor_msgs::CompressedImage& msg) {
    ImageData out;
    if (msg.has_header()) {
        if (msg.header().has_stamp()) {
            out.timestamp = StampToSec(msg.header().stamp());
        }
        out.frame_id = msg.header().frame_id();
    }
    if (msg.data().empty()) {
        return out;
    }
    const auto* bytes = reinterpret_cast<const uint8_t*>(msg.data().data());
    const std::vector<uint8_t> buf(bytes, bytes + msg.data().size());
    // IMREAD_COLOR → BGR; grayscale sources still decode to 3ch unless
    // UNCHANGED.
    cv::Mat decoded = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    if (decoded.empty()) {
        return out;
    }
    out.image = decoded;
    std::string src = "bgr8";
    if (decoded.type() == CV_8UC1) {
        src = "mono8";
    } else if (decoded.type() == CV_8UC4) {
        src = "bgra8";
    } else if (decoded.type() == CV_8UC3) {
        src = "bgr8";
    }
    if (!NormalizeImage(&out.image, src, &out.encoding)) {
        out.image.release();
    }
    return out;
}

::automsgs::msgs::sensor_msgs::Image ToProto(const ImageData& data) {
    std::string enc = data.encoding;
    if (enc.empty()) {
        enc = (data.image.channels() == 1) ? "mono8" : "bgr8";
    }
    return EncodeMat(data.image, data.timestamp, data.frame_id, enc);
}

::automsgs::msgs::sensor_msgs::Image ToProto(const cv::Mat& image,
                                             double timestamp_sec,
                                             const std::string& frame_id) {
    if (image.empty()) {
        return EncodeMat(image, timestamp_sec, frame_id, "bgr8");
    }
    if (image.type() == CV_8UC1) {
        return EncodeMat(image, timestamp_sec, frame_id, "mono8");
    }
    if (image.type() == CV_8UC3) {
        return EncodeMat(image, timestamp_sec, frame_id, "bgr8");
    }
    // Unsupported type → empty payload with header only.
    return EncodeMat(cv::Mat(), timestamp_sec, frame_id, "bgr8");
}

::automsgs::msgs::sensor_msgs::CompressedImage ToCompressedProto(
    const ImageData& data, const std::string& format, int jpeg_quality) {
    return ToCompressedProto(data.image, data.timestamp, data.frame_id, format,
                             jpeg_quality);
}

::automsgs::msgs::sensor_msgs::CompressedImage ToCompressedProto(
    const cv::Mat& image, double timestamp_sec, const std::string& frame_id,
    const std::string& format, int jpeg_quality) {
    ::automsgs::msgs::sensor_msgs::CompressedImage out;
    FillHeader(out.mutable_header(), timestamp_sec, frame_id);
    if (image.empty()) {
        return out;
    }
    std::string ext = format;
    if (!ext.empty() && ext[0] != '.') {
        ext = "." + ext;
    }
    if (ext == ".jpg") {
        ext = ".jpeg";
    }
    out.set_format(ext.size() > 1 ? ext.substr(1) : format);

    std::vector<int> params;
    if (ext == ".jpeg" || ext == ".jpg") {
        params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    } else if (ext == ".png") {
        params = {cv::IMWRITE_PNG_COMPRESSION, 3};
    }
    std::vector<uint8_t> buf;
    if (!cv::imencode(ext, image, buf, params)) {
        return out;
    }
    out.set_data(buf.data(), buf.size());
    return out;
}

}  // namespace sensor
}  // namespace argus
