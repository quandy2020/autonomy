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

#ifndef AUTONOMY_AUDIO_COMMON_MESSAGE_PROCESS_HPP_
#define AUTONOMY_AUDIO_COMMON_MESSAGE_PROCESS_HPP_

#include "autonomy/audio/common/audio_info.hpp"
#include "autonomy/audio/inference/asr/asr_engine.hpp"
#include "autonomy/audio/inference/direction_detection.hpp"
#include "autonomy/audio/inference/moving_detection.hpp"
#include "autonomy/audio/proto/asr.pb.h"
#include "autonomy/audio/proto/audio.pb.h"
#include "autonomy/audio/proto/audio_detection.pb.h"
#include "autonomy/audio/proto/microphone.pb.h"

namespace autonomy {
namespace audio {

class MessageProcess {
public:
    MessageProcess() = delete;

    // Returns true when ASR produced a non-empty final hypothesis in
    // asr_result (only meaningful when options.enable_asr() and engine ok).
    static bool OnMicrophone(const proto::AudioData& audio_data,
                             const proto::AudioOptions& options,
                             AudioInfo* audio_info,
                             DirectionDetection* direction_detection,
                             MovingDetection* moving_detection,
                             AsrEngine* asr_engine,
                             proto::AudioDetection* audio_detection,
                             proto::AsrResult* asr_result);
};

}  // namespace audio
}  // namespace autonomy

#endif  // AUTONOMY_AUDIO_COMMON_MESSAGE_PROCESS_HPP_
