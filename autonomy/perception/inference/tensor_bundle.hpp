#ifndef AUTONOMY_PERCEPTION_INFERENCE_TENSOR_BUNDLE_HPP_
#define AUTONOMY_PERCEPTION_INFERENCE_TENSOR_BUNDLE_HPP_

#include <string>
#include <vector>
#include "autonomy/common/network/common/tensor.hpp"

namespace autonomy::perception::inference {

// Every declared port must exist in the loaded graph. Types and shapes still
// come from the backend metadata, which also describes dynamic dimensions.
bool ValidateTensorPorts(
    const std::vector<std::string>& ports,
    const std::vector<common::network::ModelTensorInfo>& metadata,
    std::string* error);

}  // namespace autonomy::perception::inference

#endif
