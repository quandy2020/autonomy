#include "autonomy/perception/inference/model_session.hpp"

#include "autonomy/common/network/backend/engine.hpp"

namespace autonomy::perception::inference {

std::shared_ptr<common::network::Engine> LoadModelSession(
    const std::string& path, const std::string& backend, std::string* error) {
    return common::network::Engine::CreateEngine(path, backend, error);
}

}  // namespace autonomy::perception::inference
