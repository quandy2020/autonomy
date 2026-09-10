/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_AUDIO_INFERENCE_ASR_ASR_ENGINE_HPP_
#define AUTONOMY_AUDIO_INFERENCE_ASR_ASR_ENGINE_HPP_

#include "autonomy/audio/proto/asr.pb.h"
#include "autonomy/audio/proto/audio.pb.h"

#include <memory>
#include <string>
#include <vector>

namespace autonomy {
namespace audio {

struct AsrUtterance {
    std::vector<float> samples;  // mono, normalized roughly to [-1, 1]
    int sample_rate = 16000;
};

class AsrEngine {
public:
    virtual ~AsrEngine() = default;

    virtual bool available() const = 0;

    // Returns true when a non-empty hypothesis is produced.
    virtual bool Recognize(const AsrUtterance& utterance,
                           proto::AsrResult* result,
                           std::string* error = nullptr) = 0;
};

// Creates Sherpa-ONNX engine when compiled with AUTONOMY_HAS_SHERPA_ONNX and
// model paths are valid; otherwise a no-op stub (available() == false).
std::unique_ptr<AsrEngine> CreateAsrEngine(const proto::AudioOptions& options,
                                           std::string* error = nullptr);

}  // namespace audio
}  // namespace autonomy

#endif  // AUTONOMY_AUDIO_INFERENCE_ASR_ASR_ENGINE_HPP_
