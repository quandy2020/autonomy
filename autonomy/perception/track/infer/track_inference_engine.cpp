/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Optional ONNX track inference engine (simple + planar MINCO).
 */

#include "autonomy/perception/track/infer/track_inference_engine.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/track/common/constants.hpp"

#if defined(AUTONOMY_HAS_ONNXRUNTIME)
#include "autonomy/common/network/common/options.hpp"
#endif

namespace autonomy::perception::track {
namespace {

#if defined(AUTONOMY_HAS_ONNXRUNTIME)

std::vector<float> BuildObservationFeature(
    const std::vector<float>& robot_state, int lattice_size) {
  std::vector<float> feature(
      static_cast<size_t>(kObservationChannelCount * lattice_size), 0.0f);
  const float linear_velocity_normalized =
      robot_state.size() > 0 ? robot_state[0] : 0.0f;
  const float yaw_rate_normalized =
      robot_state.size() > 1 ? robot_state[1] : 0.0f;
  const float goal_x = robot_state.size() > 2 ? robot_state[2] : 1.0f;
  const float goal_y = robot_state.size() > 3 ? robot_state[3] : 0.0f;
  const float channel_values[kObservationChannelCount] = {
      linear_velocity_normalized, yaw_rate_normalized, goal_x, goal_y};
  for (int channel = 0; channel < kObservationChannelCount; ++channel) {
    for (int cell = 0; cell < lattice_size; ++cell) {
      feature[static_cast<size_t>(channel * lattice_size + cell)] =
          channel_values[channel];
    }
  }
  return feature;
}

double Softplus(double value) {
  return std::log1p(std::exp(std::min(value, kSoftplusClamp)));
}

double Sigmoid(double value) { return 1.0 / (1.0 + std::exp(-value)); }

bool DecodeSimplePrediction(const std::vector<float>& prediction,
                            const Lattice& lattice,
                            std::vector<MotionPrimitiveHypothesis>* hypotheses,
                            std::string* error_message) {
  const int lattice_size = lattice.size();
  if (static_cast<int>(prediction.size()) <
      kSimplePredictionChannelCount * lattice_size) {
    if (error_message) {
      *error_message = "Simple prediction size mismatch";
    }
    return false;
  }
  hypotheses->resize(static_cast<size_t>(lattice_size));
  for (int lattice_index = 0; lattice_index < lattice_size; ++lattice_index) {
    auto channel_at = [&](int channel) {
      return prediction[static_cast<size_t>(channel * lattice_size +
                                            lattice_index)];
    };
    MotionPrimitiveHypothesis hypothesis;
    hypothesis.lattice_index = lattice_index;
    hypothesis.uses_minco = false;
    lattice.DecodeTerminalState(
        lattice_index, std::tanh(static_cast<double>(channel_at(0))),
        std::tanh(static_cast<double>(channel_at(1))),
        std::tanh(static_cast<double>(channel_at(2))),
        std::tanh(static_cast<double>(channel_at(3))),
        &hypothesis.terminal_x_m, &hypothesis.terminal_y_m,
        &hypothesis.linear_velocity_mps, &hypothesis.yaw_rate_rps);
    hypothesis.trajectory_cost =
        Softplus(static_cast<double>(channel_at(4)));
    hypothesis.objectness_score =
        Sigmoid(static_cast<double>(channel_at(5)));
    (*hypotheses)[static_cast<size_t>(lattice_index)] = hypothesis;
  }
  return true;
}

bool DecodeMincoPrediction(const std::vector<float>& prediction,
                           const Lattice& lattice,
                           const proto::TrackOptions& options,
                           const std::vector<float>& robot_state,
                           std::vector<MotionPrimitiveHypothesis>* hypotheses,
                           std::string* error_message) {
  const int lattice_size = lattice.size();
  if (static_cast<int>(prediction.size()) <
      kMincoPredictionChannelCount * lattice_size) {
    if (error_message) {
      *error_message = "MINCO prediction size mismatch";
    }
    return false;
  }

  const double vel_max = options.vel_max_mps();
  const double acc_max = options.acc_max_mps2();
  const double piece_duration =
      std::max(options.minco_piece_duration_s(), kMincoMinPieceDurationS);
  const double head_vx =
      (robot_state.empty() ? 0.0 : static_cast<double>(robot_state[0])) *
      vel_max;

  hypotheses->resize(static_cast<size_t>(lattice_size));
  for (int lattice_index = 0; lattice_index < lattice_size; ++lattice_index) {
    auto channel_at = [&](int channel) {
      return prediction[static_cast<size_t>(channel * lattice_size +
                                            lattice_index)];
    };
    MotionPrimitiveHypothesis hypothesis;
    hypothesis.lattice_index = lattice_index;
    hypothesis.uses_minco = true;

    double inner_x = 0.0;
    double inner_y = 0.0;
    double unused_vx = 0.0;
    double unused_wz = 0.0;
    lattice.DecodeTerminalState(
        lattice_index, std::tanh(static_cast<double>(channel_at(0))),
        std::tanh(static_cast<double>(channel_at(1))), 0.0, 0.0, &inner_x,
        &inner_y, &unused_vx, &unused_wz);

    double tail_x = 0.0;
    double tail_y = 0.0;
    lattice.DecodeTerminalState(
        lattice_index, std::tanh(static_cast<double>(channel_at(2))),
        std::tanh(static_cast<double>(channel_at(3))), 0.0, 0.0, &tail_x,
        &tail_y, &unused_vx, &unused_wz);

    MincoBoundary& minco = hypothesis.minco;
    minco.head.velocity_x_mps = head_vx;
    minco.inner_x_m = inner_x;
    minco.inner_y_m = inner_y;
    minco.tail.position_x_m = tail_x;
    minco.tail.position_y_m = tail_y;
    minco.tail.velocity_x_mps =
        std::tanh(static_cast<double>(channel_at(4))) * vel_max;
    minco.tail.velocity_y_mps =
        std::tanh(static_cast<double>(channel_at(5))) * vel_max;
    minco.tail.acceleration_x_mps2 =
        std::tanh(static_cast<double>(channel_at(6))) * acc_max;
    minco.tail.acceleration_y_mps2 =
        std::tanh(static_cast<double>(channel_at(7))) * acc_max;
    minco.durations_s[0] = std::max(
        kMincoMinPieceDurationS,
        (std::tanh(static_cast<double>(channel_at(8))) + 1.0) * piece_duration);
    minco.durations_s[1] = std::max(
        kMincoMinPieceDurationS,
        (std::tanh(static_cast<double>(channel_at(9))) + 1.0) * piece_duration);

    hypothesis.terminal_x_m = tail_x;
    hypothesis.terminal_y_m = tail_y;
    hypothesis.linear_velocity_mps = minco.tail.velocity_x_mps;
    hypothesis.yaw_rate_rps = 0.0;
    hypothesis.trajectory_cost =
        Softplus(static_cast<double>(channel_at(10)));
    hypothesis.objectness_score =
        Sigmoid(static_cast<double>(channel_at(11)));
    (*hypotheses)[static_cast<size_t>(lattice_index)] = hypothesis;
  }
  return true;
}

#endif  // AUTONOMY_HAS_ONNXRUNTIME

}  // namespace

bool TrackInferenceEngine::IsModelLoaded() const {
#if defined(AUTONOMY_HAS_ONNXRUNTIME)
  return session_ != nullptr;
#else
  return false;
#endif
}

bool TrackInferenceEngine::LoadModel(const proto::TrackOptions& options,
                                     std::string* error_message) {
  options_ = options;
  input_height_ = std::max(1, options.image_height());
  input_width_ = std::max(1, options.image_width());
  use_minco_ = IsMincoTrajectoryMode(options.trajectory_mode());
  prediction_channel_count_ =
      PredictionChannelCountForMode(options.trajectory_mode());

#if !defined(AUTONOMY_HAS_ONNXRUNTIME)
  if (error_message) {
    *error_message =
        "ONNX Runtime not enabled in this build (AUTONOMY_HAS_ONNXRUNTIME)";
  }
  return false;
#else
  if (options.model_path().empty()) {
    if (error_message) {
      *error_message = "Track model_path is empty";
    }
    session_.reset();
    return false;
  }

  common::network::InferenceOptions inference_options;
  inference_options.model_path = options.model_path();
  inference_options.backend_id =
      options.backend_id().empty() ? "onnx" : options.backend_id();
  std::string load_error;
  session_ =
      common::network::Engine::CreateEngine(inference_options, &load_error);
  if (!session_) {
    if (error_message) {
      *error_message = load_error.empty()
                           ? "Failed to create track inference engine"
                           : load_error;
    }
    AWARN << "TrackInferenceEngine: "
          << (error_message ? *error_message : load_error);
    return false;
  }
  AINFO << "TrackInferenceEngine loaded model: " << options.model_path()
        << " (mode=" << options.trajectory_mode()
        << ", channels=" << prediction_channel_count_ << ")";
  return true;
#endif
}

bool TrackInferenceEngine::RunInference(
    const std::vector<float>& normalized_depth,
    const std::vector<float>& robot_state, const Lattice& lattice,
    std::vector<MotionPrimitiveHypothesis>* hypotheses,
    std::string* error_message) {
  if (hypotheses == nullptr) {
    return false;
  }
  hypotheses->clear();

#if !defined(AUTONOMY_HAS_ONNXRUNTIME)
  (void)normalized_depth;
  (void)robot_state;
  (void)lattice;
  if (error_message) {
    *error_message = "ONNX Runtime not enabled";
  }
  return false;
#else
  if (!session_) {
    if (error_message) {
      *error_message = "Inference session not loaded";
    }
    return false;
  }
  if (static_cast<int>(normalized_depth.size()) <
      input_height_ * input_width_) {
    if (error_message) {
      *error_message = "Depth tensor too small";
    }
    return false;
  }
  const int lattice_size = lattice.size();
  if (lattice_size <= 0) {
    if (error_message) {
      *error_message = "Empty lattice";
    }
    return false;
  }

  common::network::FloatTensorMap inputs;
  inputs[kOnnxDepthInputName] = normalized_depth;
  if (static_cast<int>(inputs[kOnnxDepthInputName].size()) >
      input_height_ * input_width_) {
    inputs[kOnnxDepthInputName].resize(
        static_cast<size_t>(input_height_ * input_width_));
  }
  inputs[kOnnxObservationInputName] =
      BuildObservationFeature(robot_state, lattice_size);

  common::network::FloatTensorMap outputs;
  if (!session_->Run(inputs, &outputs)) {
    if (error_message) {
      *error_message = session_->GetLastError();
    }
    return false;
  }

  auto prediction_it = outputs.find(kOnnxPredictionOutputName);
  if (prediction_it == outputs.end() || prediction_it->second.empty()) {
    if (error_message) {
      *error_message =
          std::string("Missing ONNX output '") + kOnnxPredictionOutputName +
          "'";
    }
    return false;
  }

  if (use_minco_) {
    return DecodeMincoPrediction(prediction_it->second, lattice, options_,
                                 robot_state, hypotheses, error_message);
  }
  return DecodeSimplePrediction(prediction_it->second, lattice, hypotheses,
                                error_message);
#endif
}

}  // namespace autonomy::perception::track
