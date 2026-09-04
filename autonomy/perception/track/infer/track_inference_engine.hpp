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
 * @brief Optional ONNX wrapper for track inference (requires BUILD_ONNXRUNTIME).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/perception/proto/track_options.pb.h"
#include "autonomy/perception/track/common/constants.hpp"
#include "autonomy/perception/track/common/types.hpp"
#include "autonomy/perception/track/primitive/lattice.hpp"

#if defined(AUTONOMY_HAS_ONNXRUNTIME)
#include "autonomy/common/network/backend/engine.hpp"
#endif

namespace autonomy::perception::track {

/**
 * @class autonomy::perception::track::TrackInferenceEngine
 * @brief Loads an exported track ONNX model and runs depth+state inference.
 *
 * Supports trajectory_mode "simple" (6-ch) and "minco" (12-ch planar MINCO).
 */
class TrackInferenceEngine {
 public:
  TrackInferenceEngine() = default;

  bool LoadModel(const proto::TrackOptions& options, std::string* error_message);
  bool IsModelLoaded() const;

  bool RunInference(const std::vector<float>& normalized_depth,
                    const std::vector<float>& robot_state,
                    const Lattice& lattice,
                    std::vector<MotionPrimitiveHypothesis>* hypotheses,
                    std::string* error_message);

 private:
#if defined(AUTONOMY_HAS_ONNXRUNTIME)
  std::unique_ptr<common::network::Engine> session_;
#endif

  proto::TrackOptions options_;
  int input_height_{96};
  int input_width_{160};
  int prediction_channel_count_{kSimplePredictionChannelCount};
  bool use_minco_{false};
};

}  // namespace autonomy::perception::track
