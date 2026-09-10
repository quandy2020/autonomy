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

#include "autonomy/audio/audio_component.hpp"

#include "autonomy/audio/common/message_process.hpp"
#include "autonomy/audio/options.hpp"
#include "autonomy/common/logging.hpp"

#include <utility>

namespace autonomy {
namespace audio {

AudioComponent::~AudioComponent() { Clear(); }

bool AudioComponent::Init() {
    proto::AudioOptions options;
    if (!GetProtoConfig(&options)) {
        AERROR << "Audio component failed to load config from '"
               << ConfigFilePath() << "'.";
        return false;
    }
    std::string error;
    if (!ValidateAudioOptions(options, &error)) {
        AERROR << error;
        Clear();
        return false;
    }
    options_ = std::move(options);
    if (!options_.enable_direction() && !options_.enable_moving() &&
        !options_.enable_asr()) {
        options_.set_enable_direction(true);
        options_.set_enable_moving(true);
    }
    if (options_.cache_signal_time_sec() <= 0) {
        options_.set_cache_signal_time_sec(3);
    }

    audio_info_.set_cache_signal_time_sec(options_.cache_signal_time_sec());

    audio_writer_ = node_->CreateWriter<proto::AudioDetection>(
        options_.audio_detection_topic());
    if (!audio_writer_) {
        AERROR << "Audio failed to create writer on "
               << options_.audio_detection_topic();
        Clear();
        return false;
    }

    if (options_.enable_asr()) {
        std::string asr_error;
        asr_engine_ = CreateAsrEngine(options_, &asr_error);
        if (asr_engine_ == nullptr || !asr_engine_->available()) {
            AWARN << "ASR enabled but backend unavailable"
                  << (asr_error.empty() ? "." : (": " + asr_error));
        } else {
            asr_writer_ = node_->CreateWriter<proto::AsrResult>(
                options_.asr_result_topic());
            if (!asr_writer_) {
                AERROR << "Audio failed to create ASR writer on "
                       << options_.asr_result_topic();
                Clear();
                return false;
            }
            if (!options_.speech_recognition_topic().empty()) {
                speech_writer_ =
                    node_->CreateWriter<automsgs::msgs::std_msgs::String>(
                        options_.speech_recognition_topic());
                if (!speech_writer_) {
                    AERROR << "Audio failed to create speech writer on "
                           << options_.speech_recognition_topic();
                    Clear();
                    return false;
                }
            }
            AINFO << "ASR ready → " << options_.asr_result_topic();
        }
    }

    AINFO << "Audio component ready: in=" << options_.audio_data_topic()
          << " out=" << options_.audio_detection_topic();
    return true;
}

bool AudioComponent::Proc(
    const std::shared_ptr<proto::AudioData>& audio_data) {
    if (audio_data == nullptr) {
        AERROR << "Audio received a null AudioData.";
        return false;
    }

    proto::AudioDetection audio_detection;
    audio_detection.mutable_header()->CopyFrom(audio_data->header());
    if (!audio_data->microphone_config().frame_id().empty()) {
        audio_detection.mutable_header()->set_frame_id(
            audio_data->microphone_config().frame_id());
    }

    proto::AsrResult asr_result;
    const bool asr_hit = MessageProcess::OnMicrophone(
        *audio_data, options_, &audio_info_, &direction_detection_,
        &moving_detection_, asr_engine_.get(), &audio_detection, &asr_result);

    if (!audio_writer_->Write(audio_detection)) {
        AERROR << "Audio failed to publish AudioDetection.";
        return false;
    }

    if (asr_hit && asr_writer_) {
        if (!asr_writer_->Write(asr_result)) {
            AERROR << "Audio failed to publish AsrResult.";
            return false;
        }
        if (speech_writer_) {
            automsgs::msgs::std_msgs::String text;
            text.set_data(asr_result.text());
            if (!speech_writer_->Write(text)) {
                AERROR << "Audio failed to publish speech_recognition text.";
                return false;
            }
        }
    }
    return true;
}

void AudioComponent::Clear() {
    speech_writer_.reset();
    asr_writer_.reset();
    audio_writer_.reset();
    asr_engine_.reset();
}

}  // namespace audio
}  // namespace autonomy

AUTOLINK_REGISTER_COMPONENT(autonomy::audio::AudioComponent)
