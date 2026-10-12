/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file mpcnet_onnx_controller.hpp
 * @brief Convenience function for creating an environment for ONNX Runtime and getting a corresponding shared pointer.
 */


#include <onnxruntime/onnxruntime_cxx_api.h>

#include <bolza/oc/synchronized_module/reference_manager_interface.hpp>

#include "bolza/mpcnet/mpcnet_definition_base.hpp"
#include "bolza/mpcnet/control/mpcnet_controller_base.hpp"

namespace bolza {
namespace mpcnet {

/**
 * Convenience function for creating an environment for ONNX Runtime and getting a corresponding shared pointer.
 * @note Only one environment per process can be created. The environment offers some threading and logging options.
 * @return Pointer to the environment for ONNX Runtime.
 */
inline std::shared_ptr<Ort::Env> CreateOnnxEnvironment() {
  return std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "MPCNetOnnxController");
}

/**
 * A neural network controller using ONNX Runtime based on the Open Neural Network Exchange (ONNX) format.
 * The model of the policy computes u = model(t, x) with
 * t: generalized time (1 x dimensionOfTime),
 * x: relative state (1 x dimensionOfState),
 * u: predicted input (1 x dimensionOfInput),
 * @note The additional first dimension with size 1 for the variables of the model comes from batch processing during training.
 */
class MPCNetOnnxController final : public MPCNetControllerBase {
 public:
  /**
   * Constructor.
   * @note The class is not fully instantiated until calling LoadPolicyModel().
   * @param [in] mpcnetDefinitionPtr: Pointer to the MPC-Net definitions.
   * @param [in] referenceManagerPtr: Pointer to the reference manager.
   * @param [in] onnxEnvironmentPtr: Pointer to the environment for ONNX Runtime.
   */
  MPCNetOnnxController(std::shared_ptr<MPCNetDefinitionBase> mpcnetDefinitionPtr,
                       std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr, std::shared_ptr<Ort::Env> onnxEnvironmentPtr)
      /**
       * @brief Mpcnet Definition Ptr.
       * @return Result value.
       */
      : mpcnetDefinitionPtr_(std::move(mpcnetDefinitionPtr)),
        referenceManagerPtr_(std::move(referenceManagerPtr)),
        onnxEnvironmentPtr_(std::move(onnxEnvironmentPtr)) {}

  /**
   * @brief Destructor.
   */
  ~MPCNetOnnxController() override = default;
  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  MPCNetOnnxController* Clone() const override { return new MPCNetOnnxController(*this); }

  /**
   * @brief Load Policy Model.
   * @param [in] policyFilePath: Policy File Path.
   */
  void LoadPolicyModel(const std::string& policyFilePath) override;

  /**
   * @brief Compute Input.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  vector_t ComputeInput(const scalar_t t, const vector_t& x) override;
  /**
   * @brief Get Type.
   * @return Result value.
   */
  ControllerType GetType() const override { return ControllerType::ONNX; }

  /**
   * @brief Size.
   * @return Result value.
   */
  int size() const override { throw std::runtime_error("[MPCNetOnnxController::size] not implemented."); }
  /**
   * @brief Clear.
   */
  void clear() override { throw std::runtime_error("[MPCNetOnnxController::clear] not implemented."); }
  /**
   * @brief Empty.
   * @return True on success.
   */
  bool empty() const override { throw std::runtime_error("[MPCNetOnnxController::empty] not implemented."); }
  /**
   * @brief Concatenate.
   * @param [in] otherController: Other Controller.
   * @param [in] index: Index.
   * @param [in] length: Length.
   */
  void Concatenate(const ControllerBase* otherController, int index, int length) override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[MPCNetOnnxController::Concatenate] not implemented.");
  }

 private:
  using tensor_element_t = float;

  MPCNetOnnxController(const MPCNetOnnxController& other)
      /**
       * @brief MPCNet Onnx Controller.
       * @param [in] mpcnetDefinitionPtr_: Mpcnet Definition Ptr.
       * @param [in] referenceManagerPtr_: Reference Manager Ptr.
       * @param [in] onnxEnvironmentPtr_: Onnx Environment Ptr.
       * @return Result value.
       */
      : MPCNetOnnxController(other.mpcnetDefinitionPtr_, other.referenceManagerPtr_, other.onnxEnvironmentPtr_) {
    if (!other.policyFilePath_.empty()) {
      /**
       * @brief Constructs a LoadPolicyModel.
       * @param [in] policyFilePath_: Policy File Path.
       */
      LoadPolicyModel(other.policyFilePath_);
    }
  }

  std::shared_ptr<MPCNetDefinitionBase> mpcnetDefinitionPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
  std::shared_ptr<Ort::Env> onnxEnvironmentPtr_;
  std::string policyFilePath_;
  std::unique_ptr<Ort::Session> sessionPtr_;
  std::vector<const char*> inputNames_;
  std::vector<const char*> outputNames_;
  std::vector<std::vector<int64_t>> inputShapes_;
  std::vector<std::vector<int64_t>> outputShapes_;
};

}  // namespace mpcnet
}  // namespace bolza
