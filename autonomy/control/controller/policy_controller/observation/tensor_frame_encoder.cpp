/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/tensor_frame_encoder.hpp"

#include <algorithm>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

std::size_t ExpectedElements(const proto::ObservationInput& input,
                             std::size_t fallback) {
    if (input.shape_size() == 0) {
        return fallback;
    }
    std::size_t count = 1;
    bool any = false;
    for (int i = 0; i < input.shape_size(); ++i) {
        if (input.shape(i) > 0) {
            count *= static_cast<std::size_t>(input.shape(i));
            any = true;
        }
    }
    return any ? count : fallback;
}

bool CopyOrResize(const TensorFrame& frame, const proto::ObservationInput& input,
                  std::vector<float>* output, std::string* error,
                  const char* modality_label) {
    if (output == nullptr) {
        return false;
    }
    if (frame.data.empty()) {
        if (error != nullptr) {
            *error = std::string(modality_label) + " frame is empty";
        }
        return false;
    }
    const std::size_t need =
        ExpectedElements(input, frame.data.size());
    if (need == frame.data.size()) {
        *output = frame.data;
        return true;
    }
    output->assign(need, 0.0f);
    const std::size_t copy = std::min(need, frame.data.size());
    std::copy_n(frame.data.begin(), copy, output->begin());
    return true;
}

}  // namespace

bool EncodeLidarBev(const ObservationSnapshot& snapshot,
                    const proto::ObservationInput& input,
                    std::vector<float>* output, std::string* error) {
    if (snapshot.lidar_bev == nullptr) {
        if (error != nullptr) {
            *error = "Lidar-BEV modality requires SetLidarBev";
        }
        return false;
    }
    return CopyOrResize(*snapshot.lidar_bev, input, output, error, "Lidar-BEV");
}

bool EncodeRawTensor(const ObservationSnapshot& snapshot,
                     const proto::ObservationInput& input,
                     std::vector<float>* output, std::string* error) {
    if (input.name().empty()) {
        if (error != nullptr) {
            *error = "RAW modality requires ObservationInput.name";
        }
        return false;
    }
    const auto it = snapshot.raw_tensors.find(input.name());
    if (it == snapshot.raw_tensors.end() || it->second == nullptr) {
        if (error != nullptr) {
            *error = "Missing RAW tensor: " + input.name();
        }
        return false;
    }
    return CopyOrResize(*it->second, input, output, error, "RAW");
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
