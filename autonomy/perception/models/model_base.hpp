#ifndef AUTONOMY_PERCEPTION_MODELS_MODEL_BASE_HPP_
#define AUTONOMY_PERCEPTION_MODELS_MODEL_BASE_HPP_

#include <string>
#include <vector>

#include "autonomy/common/network/common/tensor.hpp"

namespace autonomy::perception::models {

class ModelBase {
public:
    virtual ~ModelBase() = default;
    virtual const std::string& name() const noexcept = 0;
    virtual std::vector<common::network::ModelTensorInfo> inputs() const = 0;
    virtual std::vector<common::network::ModelTensorInfo> supported_outputs() const = 0;
    virtual bool Infer(const common::network::TensorMap& inputs,
                       common::network::TensorMap* outputs,
                       std::string* error) = 0;
};

}  // namespace autonomy::perception::models

#endif
