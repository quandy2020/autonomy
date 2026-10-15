/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/image_encoder.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "autonomy/common/network/detail/preprocess/image.hpp"
#include "autonomy/common/network/detail/preprocess/policy.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

bool ImageToBgr(const automsgs::msgs::sensor_msgs::Image& image, cv::Mat* bgr,
                std::string* error) {
    if (bgr == nullptr || image.width() == 0 || image.height() == 0 ||
        image.data().empty()) {
        if (error != nullptr) {
            *error = "Empty image";
        }
        return false;
    }
    const int width = static_cast<int>(image.width());
    const int height = static_cast<int>(image.height());
    const std::string& encoding = image.encoding();
    auto* raw = reinterpret_cast<unsigned char*>(
        const_cast<char*>(image.data().data()));
    if (encoding == "bgr8" || encoding == "BGR8") {
        *bgr = cv::Mat(height, width, CV_8UC3, raw,
                       image.step() > 0 ? image.step()
                                        : static_cast<size_t>(width * 3))
                   .clone();
        return true;
    }
    if (encoding == "rgb8" || encoding == "RGB8") {
        cv::Mat rgb(height, width, CV_8UC3, raw,
                    image.step() > 0 ? image.step()
                                     : static_cast<size_t>(width * 3));
        cv::cvtColor(rgb, *bgr, cv::COLOR_RGB2BGR);
        return true;
    }
    if (encoding == "mono8" || encoding == "MONO8") {
        cv::Mat gray(height, width, CV_8UC1, raw,
                     image.step() > 0 ? image.step()
                                      : static_cast<size_t>(width));
        cv::cvtColor(gray, *bgr, cv::COLOR_GRAY2BGR);
        return true;
    }
    if (error != nullptr) {
        *error = "Unsupported image encoding: " + encoding;
    }
    return false;
}

}  // namespace

bool EncodeImage(const ObservationSnapshot& snapshot,
                 const proto::ObservationInput& input,
                 std::vector<float>* output, std::string* error) {
    if (output == nullptr) {
        return false;
    }
    if (snapshot.image == nullptr) {
        if (error != nullptr) {
            *error = "Image modality requires an image";
        }
        return false;
    }

    cv::Mat bgr;
    if (!ImageToBgr(*snapshot.image, &bgr, error)) {
        return false;
    }

    int height = bgr.rows;
    int width = bgr.cols;
    int channels = 3;
    if (input.shape_size() >= 2) {
        const int rank = input.shape_size();
        if (rank >= 4 && input.shape(1) > 0 &&
            input.shape(2) > 0 && input.shape(3) > 0) {
            channels = static_cast<int>(input.shape(1));
            height = static_cast<int>(input.shape(2));
            width = static_cast<int>(input.shape(3));
        } else if (rank >= 3 && input.shape(rank - 3) > 0 &&
                   input.shape(rank - 2) > 0 &&
                   input.shape(rank - 1) > 0) {
            channels = static_cast<int>(input.shape(rank - 3));
            height = static_cast<int>(input.shape(rank - 2));
            width = static_cast<int>(input.shape(rank - 1));
        } else {
            if (input.shape(rank - 2) > 0) {
                height = static_cast<int>(input.shape(rank - 2));
            }
            if (input.shape(rank - 1) > 0) {
                width = static_cast<int>(input.shape(rank - 1));
            }
        }
    }

    ::autonomy::common::network::ModelTensorInfo metadata;
    metadata.name = input.name();
    metadata.shape = ::autonomy::common::network::TensorShape(
        std::vector<int64_t>{1, channels, height, width});
    metadata.element_type = ::autonomy::common::network::ElementType::kFloat32;

    ::autonomy::common::network::PreprocessOptions preprocess =
        ::autonomy::common::network::Letterbox<224, 224>();
    preprocess.default_height = height;
    preprocess.default_width = width;
    if (input.preprocess() == "stretch") {
        preprocess = ::autonomy::common::network::Stretch(height, width);
    } else if (input.preprocess() == "center_crop") {
        preprocess = ::autonomy::common::network::CenterCrop(height, width);
    } else {
        preprocess.resize = ::autonomy::common::network::ResizePolicy::kLetterbox;
        preprocess.normalize = ::autonomy::common::network::NormalizePolicy::kZeroOne;
        preprocess.default_height = height;
        preprocess.default_width = width;
    }

    if (!::autonomy::common::network::Preprocess(bgr, metadata, preprocess, output, nullptr,
                                     error)) {
        if (error != nullptr && error->empty()) {
            *error = "Image preprocess failed for " + input.name();
        }
        return false;
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
