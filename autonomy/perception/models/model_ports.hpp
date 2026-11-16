#ifndef AUTONOMY_PERCEPTION_MODELS_MODEL_PORTS_HPP_
#define AUTONOMY_PERCEPTION_MODELS_MODEL_PORTS_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace autonomy::perception::models {

// Minimum number of published named ports for each graph stage. Split
// exports explicitly choose the ports and their names in the stage binding.
struct ModelPorts {
    std::size_t minimum_inputs;
    std::size_t minimum_outputs;
};

inline bool CheckPortCount(const ModelPorts& ports,
                           const std::vector<std::string>& inputs,
                           const std::vector<std::string>& outputs) {
    return inputs.size() >= ports.minimum_inputs &&
           outputs.size() >= ports.minimum_outputs;
}

}  // namespace autonomy::perception::models

#endif
