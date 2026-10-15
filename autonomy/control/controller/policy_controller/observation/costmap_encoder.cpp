/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/costmap_encoder.hpp"

#include <algorithm>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

bool EncodeCostmap(const ObservationSnapshot& snapshot,
                   const proto::ObservationInput& input,
                   std::vector<float>* output, std::string* error) {
    if (output == nullptr) {
        return false;
    }
    if (snapshot.costmap == nullptr || snapshot.costmap->data.empty()) {
        if (error != nullptr) {
            *error = "Costmap modality requires a valid local crop";
        }
        return false;
    }

    const auto& crop = *snapshot.costmap;
    int source_height = std::max(1, crop.height);
    int source_width = std::max(1, crop.width);

    int output_height = source_height;
    int output_width = source_width;
    if (input.shape_size() >= 2) {
        const int rank = input.shape_size();
        if (input.shape(rank - 2) > 0) {
            output_height = static_cast<int>(input.shape(rank - 2));
        }
        if (input.shape(rank - 1) > 0) {
            output_width = static_cast<int>(input.shape(rank - 1));
        }
    }

    // Fast path: same size, already [0,1].
    if (output_height == source_height && output_width == source_width &&
        static_cast<int>(crop.data.size()) == source_height * source_width) {
        *output = crop.data;
        return true;
    }

    output->assign(static_cast<std::size_t>(output_height * output_width), 0.0f);
    for (int y = 0; y < output_height; ++y) {
        const int source_y =
            source_height <= 1
                ? 0
                : y * (source_height - 1) / std::max(1, output_height - 1);
        for (int x = 0; x < output_width; ++x) {
            const int source_x =
                source_width <= 1
                    ? 0
                    : x * (source_width - 1) / std::max(1, output_width - 1);
            const std::size_t index = static_cast<std::size_t>(
                source_y * source_width + source_x);
            float value = 0.0f;
            if (index < crop.data.size()) {
                value = std::max(0.0f, std::min(1.0f, crop.data[index]));
            }
            (*output)[static_cast<std::size_t>(y * output_width + x)] = value;
        }
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
