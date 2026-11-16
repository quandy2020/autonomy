#include "autonomy/perception/models/segformer/segformer_model.hpp"

#include "autonomy/perception/models/model_ports.hpp"

#include <utility>

namespace autonomy::perception::models {

std::unique_ptr<ModelBase> LoadSegFormerModel(
    std::string name, const std::string& path, const std::string& backend,
    const std::vector<std::string>& inputs,
    const std::vector<std::string>& outputs, std::string* error) {
    if (!CheckPortCount({1, 1}, inputs, outputs)) {
        if (error)
            *error = "SegFormer graph stage requires named input and output bindings.";
        return nullptr;
    }
    return LoadGraphModel(std::move(name), path, backend, inputs, outputs, error);
}

}  // namespace autonomy::perception::models
