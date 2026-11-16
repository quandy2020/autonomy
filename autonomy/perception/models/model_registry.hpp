#ifndef AUTONOMY_PERCEPTION_MODELS_MODEL_REGISTRY_HPP_
#define AUTONOMY_PERCEPTION_MODELS_MODEL_REGISTRY_HPP_

#include <unordered_map>
#include "autonomy/perception/inference/model_session.hpp"
#include "autonomy/perception/models/model_base.hpp"

namespace autonomy::perception::models {

// The graph must implement the selected stage. The caller prepares its typed
// tensors and is responsible for any preprocessing between stages.
std::unique_ptr<ModelBase> LoadGraphModel(
    std::string name, const std::string& path, const std::string& backend,
    const std::vector<std::string>& input_ports,
    const std::vector<std::string>& output_ports, std::string* error);

}  // namespace autonomy::perception::models

namespace autonomy::perception::base {

using ModelSessions =
    std::unordered_map<std::string, std::shared_ptr<common::network::Engine>>;

std::shared_ptr<common::network::Engine> LoadRegisteredModel(
    const std::string& id, const std::string& path, const std::string& backend,
    ModelSessions* sessions, std::string* error);

}  // namespace autonomy::perception::base

#endif
