#include "autonomy/perception/inference/tensor_bundle.hpp"

#include <algorithm>
#include <unordered_set>

namespace autonomy::perception::inference {

bool ValidateTensorPorts(
    const std::vector<std::string>& ports,
    const std::vector<common::network::ModelTensorInfo>& metadata,
    std::string* error) {
    std::unordered_set<std::string> seen;
    for (const auto& port : ports) {
        if (port.empty() || !seen.insert(port).second ||
            std::none_of(metadata.begin(), metadata.end(),
                         [&](const auto& tensor) { return tensor.name == port; })) {
            if (error)
                *error = "Unknown or duplicate tensor port: " + port;
            return false;
        }
    }
    return true;
}

}  // namespace autonomy::perception::inference
