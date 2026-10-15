/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/model.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

PolicyModel::PolicyModel(PolicyConfig config,
                         std::unique_ptr<::autonomy::common::network::Engine> engine)
    : config_(std::move(config)), engine_(std::move(engine)) {}

PolicyModel::SharedPointer PolicyModel::Create(
    PolicyConfig config, std::string* error) {
    auto options = config.ToInferenceOptions();
    if (options.model_path.empty()) {
        if (error != nullptr) {
            *error = "PolicyConfig inference.model_path is empty";
        }
        return nullptr;
    }
    auto engine = ::autonomy::common::network::Engine::CreateEngine(options, error);
    if (!engine) {
        return nullptr;
    }
    std::string warmup_error;
    engine->Warmup(&warmup_error);
    std::string validate_error;
    if (!config.ValidateAgainstEngine(engine->GetInputInfos(),
                                             engine->GetOutputInfos(),
                                             &validate_error)) {
        if (error != nullptr) {
            *error = validate_error;
        }
        return nullptr;
    }
    return SharedPointer(
        new PolicyModel(std::move(config), std::move(engine)));
}

bool PolicyModel::RunInference(const ::autonomy::common::network::FloatTensorMap& inputs,
                               ::autonomy::common::network::FloatTensorMap* outputs,
                               std::string* error) const {
    if (outputs == nullptr || !engine_) {
        if (error != nullptr) {
            *error = "PolicyModel RunInference invalid state";
        }
        return false;
    }
    std::lock_guard<std::mutex> lock(inference_mutex_);
    if (!engine_->Run(inputs, outputs)) {
        if (error != nullptr) {
            *error = engine_->GetLastError();
            if (error->empty()) {
                *error = "Engine::Run failed";
            }
        }
        return false;
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
