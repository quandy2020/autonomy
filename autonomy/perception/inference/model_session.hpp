#ifndef AUTONOMY_PERCEPTION_INFERENCE_MODEL_SESSION_HPP_
#define AUTONOMY_PERCEPTION_INFERENCE_MODEL_SESSION_HPP_

#include <memory>
#include <string>

namespace autonomy::common::network {
class Engine;
}

namespace autonomy::perception::inference {

std::shared_ptr<common::network::Engine> LoadModelSession(
    const std::string& path, const std::string& backend, std::string* error);

}  // namespace autonomy::perception::inference

#endif  // AUTONOMY_PERCEPTION_INFERENCE_MODEL_SESSION_HPP_
