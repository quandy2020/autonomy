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

#include "autonomy/audio/common/message_process.hpp"

#include "autonomy/common/logging.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace autonomy {
namespace audio {
namespace {

double FrameRms(const std::vector<double>& samples) {
    if (samples.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double s : samples) {
        sum += s * s;
    }
    return std::sqrt(sum / static_cast<double>(samples.size()));
}

AsrUtterance ToUtterance(const std::vector<double>& pcm_i16, int sample_rate) {
    AsrUtterance u;
    u.sample_rate = sample_rate > 0 ? sample_rate : 16000;
    u.samples.reserve(pcm_i16.size());
    for (double s : pcm_i16) {
        u.samples.push_back(static_cast<float>(s / 32768.0));
    }
    return u;
}

}  // namespace

bool MessageProcess::OnMicrophone(const proto::AudioData& audio_data,
                                  const proto::AudioOptions& options,
                                  AudioInfo* audio_info,
                                  DirectionDetection* direction_detection,
                                  MovingDetection* moving_detection,
                                  AsrEngine* asr_engine,
                                  proto::AudioDetection* audio_detection,
                                  proto::AsrResult* asr_result) {
    audio_info->Insert(audio_data);

    const auto& mic = audio_data.microphone_config();
    const int chunk = mic.chunk() > 0 ? mic.chunk() : 8192;
    const int sample_rate =
        mic.sample_rate() > 0.0F ? static_cast<int>(mic.sample_rate()) : 48000;
    const double mic_distance =
        mic.mic_distance() > 0.0F ? mic.mic_distance() : 0.065;

    if (options.enable_direction() && direction_detection != nullptr &&
        audio_detection != nullptr) {
        auto direction_result = direction_detection->EstimateSoundSource(
            audio_info->GetSignals(chunk), options.respeaker_extrinsics_path(),
            sample_rate, mic_distance);
        *audio_detection->mutable_position() = direction_result.first;
        audio_detection->set_source_degree(direction_result.second);
    }

    if (options.enable_moving() && moving_detection != nullptr &&
        audio_detection != nullptr) {
        const auto signals = audio_info->GetSignals(chunk);
        audio_detection->set_moving_result(moving_detection->Detect(signals));
    }

    bool asr_ok = false;
    if (options.enable_asr() && asr_engine != nullptr &&
        asr_engine->available() && asr_result != nullptr) {
        // Use ~1s window when cache allows; else whatever is buffered.
        const int asr_window = std::max(chunk, sample_rate);
        std::vector<double> pcm;
        if (options.asr_prefer_asr_channel()) {
            pcm = audio_info->GetAsrSignal(asr_window);
        } else {
            const auto raw = audio_info->GetSignals(asr_window);
            if (!raw.empty()) {
                pcm = raw.front();
            }
        }
        const float energy_threshold = options.asr_energy_threshold() > 0.0F
                                           ? options.asr_energy_threshold()
                                           : 200.0F;
        if (!pcm.empty() && FrameRms(pcm) >= energy_threshold) {
            std::string error;
            AsrUtterance utterance = ToUtterance(pcm, sample_rate);
            if (asr_engine->Recognize(utterance, asr_result, &error)) {
                asr_result->mutable_header()->CopyFrom(audio_data.header());
                if (!mic.frame_id().empty()) {
                    asr_result->mutable_header()->set_frame_id(mic.frame_id());
                }
                asr_ok = true;
            } else if (!error.empty()) {
                ADEBUG << "ASR skip: " << error;
            }
        }
    }
    return asr_ok;
}

}  // namespace audio
}  // namespace autonomy
