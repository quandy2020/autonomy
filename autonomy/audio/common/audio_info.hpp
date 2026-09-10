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

#ifndef AUTONOMY_AUDIO_COMMON_AUDIO_INFO_HPP_
#define AUTONOMY_AUDIO_COMMON_AUDIO_INFO_HPP_

#include "autonomy/audio/proto/microphone.pb.h"

#include <deque>
#include <vector>

namespace autonomy {
namespace audio {

class AudioInfo {
public:
    AudioInfo() = default;

    void set_cache_signal_time_sec(int seconds) {
        cache_signal_time_sec_ = seconds;
    }

    void Insert(const proto::AudioData& audio_data);

    // RAW channels for direction / moving (one vector per RAW mic).
    std::vector<std::vector<double>> GetSignals(int signal_length) const;

    // Prefer CHANNEL_TYPE_ASR; otherwise first RAW. Empty if neither exists.
    std::vector<double> GetAsrSignal(int signal_length) const;

    float last_sample_rate() const { return last_sample_rate_; }

private:
    void InsertTypedChannel(std::deque<double>* dest,
                            const proto::ChannelData& channel_data,
                            const proto::MicrophoneConfig& microphone_config);

    int cache_signal_time_sec_ = 3;
    float last_sample_rate_ = 48000.0F;
    std::vector<std::deque<double>> raw_signals_;
    std::deque<double> asr_signal_;
    bool has_asr_channel_ = false;
};

}  // namespace audio
}  // namespace autonomy

#endif  // AUTONOMY_AUDIO_COMMON_AUDIO_INFO_HPP_
