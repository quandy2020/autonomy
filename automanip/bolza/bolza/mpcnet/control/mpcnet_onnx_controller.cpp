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

#include "bolza/mpcnet/control/mpcnet_onnx_controller.hpp"

namespace bolza {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetOnnxController::LoadPolicyModel(const std::string& policyFilePath) {
  policyFilePath_ = policyFilePath;
  // create session
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  sessionOptions.SetInterOpNumThreads(1);
  sessionPtr_.reset(new Ort::Session(*onnxEnvironmentPtr_, policyFilePath_.c_str(), sessionOptions));
  // get input and output info
  inputNames_.clear();
  outputNames_.clear();
  inputShapes_.clear();
  outputShapes_.clear();
  Ort::AllocatorWithDefaultOptions allocator;
  for (int i = 0; i < sessionPtr_->GetInputCount(); i++) {
    inputNames_.push_back(sessionPtr_->GetInputName(i, allocator));
    inputShapes_.push_back(sessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (int i = 0; i < sessionPtr_->GetOutputCount(); i++) {
    outputNames_.push_back(sessionPtr_->GetOutputName(i, allocator));
    outputShapes_.push_back(sessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MPCNetOnnxController::ComputeInput(const scalar_t t, const vector_t& x) {
  if (sessionPtr_ == nullptr) {
    throw std::runtime_error("[MPCNetOnnxController::ComputeInput] cannot compute input, since policy model is not loaded.");
  }
  // create input tensor object
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> observation =
      mpcnetDefinitionPtr_->getObservation(t, x, referenceManagerPtr_->getModeSchedule(), referenceManagerPtr_->getTargetTrajectories())
          .cast<tensor_element_t>();
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, observation.data(), observation.size(),
                                                                   inputShapes_[0].data(), inputShapes_[0].size()));
  // run inference
  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues = sessionPtr_->Run(runOptions, inputNames_.data(), inputValues.data(), 1, outputNames_.data(), 1);
  // evaluate output tensor object
  Eigen::Map<Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1>> action(outputValues[0].GetTensorMutableData<tensor_element_t>(),
                                                                        outputShapes_[0][1], outputShapes_[0][0]);
  std::pair<matrix_t, vector_t> actionTransformation = mpcnetDefinitionPtr_->getActionTransformation(
      t, x, referenceManagerPtr_->getModeSchedule(), referenceManagerPtr_->getTargetTrajectories());
  // transform action
  return actionTransformation.first * action.cast<scalar_t>() + actionTransformation.second;
}

}  // namespace mpcnet
}  // namespace bolza
