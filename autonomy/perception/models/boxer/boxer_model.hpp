#ifndef AUTONOMY_PERCEPTION_MODELS_BOXER_MODEL_HPP_
#define AUTONOMY_PERCEPTION_MODELS_BOXER_MODEL_HPP_

#include "autonomy/perception/models/model_registry.hpp"

namespace autonomy::perception::models {

std::unique_ptr<ModelBase> LoadBoxerModel(
    std::string name, const std::string& path, const std::string& backend,
    const std::vector<std::string>& inputs,
    const std::vector<std::string>& outputs, std::string* error);

}  // namespace autonomy::perception::models

#endif
