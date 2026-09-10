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

#include "autonomy/audio/options.hpp"

namespace autonomy {
namespace audio {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Audio: " + message;
    }
}

bool AbsoluteTopic(const std::string& topic) {
    return !topic.empty() && topic.front() == '/';
}

}  // namespace

bool ValidateAudioOptions(const proto::AudioOptions& options,
                          std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (!AbsoluteTopic(options.audio_data_topic()) ||
        !AbsoluteTopic(options.audio_detection_topic())) {
        SetError(error,
                 "audio_data_topic and audio_detection_topic must be absolute.");
        return false;
    }
    if (!options.localization_topic().empty() &&
        !AbsoluteTopic(options.localization_topic())) {
        SetError(error, "localization_topic must be absolute when set.");
        return false;
    }
    if (options.cache_signal_time_sec() <= 0) {
        SetError(error, "cache_signal_time_sec must be positive.");
        return false;
    }
    if (options.enable_asr()) {
        if (!AbsoluteTopic(options.asr_result_topic())) {
            SetError(error, "asr_result_topic must be absolute when enable_asr.");
            return false;
        }
        if (!options.speech_recognition_topic().empty() &&
            !AbsoluteTopic(options.speech_recognition_topic())) {
            SetError(error,
                     "speech_recognition_topic must be absolute when set.");
            return false;
        }
    }
    return true;
}

}  // namespace audio
}  // namespace autonomy
