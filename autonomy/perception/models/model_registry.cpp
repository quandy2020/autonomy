#include "autonomy/perception/models/model_registry.hpp"

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/perception/inference/tensor_bundle.hpp"

#include <utility>

namespace autonomy::perception::models {
namespace {

class GraphModel final : public ModelBase {
public:
    GraphModel(std::string name, std::shared_ptr<common::network::Engine> session)
        : name_(std::move(name)), session_(std::move(session)) {}

    const std::string& name() const noexcept override { return name_; }

    std::vector<common::network::ModelTensorInfo> inputs() const override {
        return session_->GetInputInfos();
    }

    std::vector<common::network::ModelTensorInfo> supported_outputs() const override {
        return session_->GetOutputInfos();
    }

    bool Infer(const common::network::TensorMap& inputs,
               common::network::TensorMap* outputs, std::string* error) override {
        if (!outputs) {
            if (error)
                *error = "Missing tensor output buffer.";
            return false;
        }
        common::network::TensorMap result;
        if (!session_->Run(inputs, &result)) {
            if (error)
                *error = session_->GetLastError();
            return false;
        }
        *outputs = std::move(result);
        return true;
    }

private:
    std::string name_;
    std::shared_ptr<common::network::Engine> session_;
};

}  // namespace

std::unique_ptr<ModelBase> LoadGraphModel(
    std::string name, const std::string& path, const std::string& backend,
    const std::vector<std::string>& input_ports,
    const std::vector<std::string>& output_ports, std::string* error) {
    auto session = inference::LoadModelSession(path, backend, error);
    if (!session ||
        !inference::ValidateTensorPorts(input_ports, session->GetInputInfos(), error) ||
        !inference::ValidateTensorPorts(output_ports, session->GetOutputInfos(), error))
        return nullptr;
    return std::make_unique<GraphModel>(std::move(name), std::move(session));
}

}  // namespace autonomy::perception::models

namespace autonomy::perception::base {

std::shared_ptr<common::network::Engine> LoadRegisteredModel(
    const std::string& id, const std::string& path, const std::string& backend,
    ModelSessions* sessions, std::string* error) {
    if (sessions && !id.empty()) {
        const auto found = sessions->find(id);
        if (found != sessions->end())
            return found->second;
    }
    auto session = inference::LoadModelSession(path, backend, error);
    if (session && sessions && !id.empty())
        sessions->emplace(id, session);
    return session;
}

}  // namespace autonomy::perception::base
