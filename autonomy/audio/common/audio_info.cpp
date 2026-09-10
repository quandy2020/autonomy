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

#include "autonomy/audio/common/audio_info.hpp"

#include <algorithm>
#include <cstdint>

namespace autonomy {
namespace audio {

void AudioInfo::Insert(const proto::AudioData& audio_data) {
    if (audio_data.microphone_config().sample_rate() > 0.0F) {
        last_sample_rate_ = audio_data.microphone_config().sample_rate();
    }

    std::size_t raw_index = 0;
    for (const auto& channel_data : audio_data.channel_data()) {
        if (channel_data.channel_type() == proto::CHANNEL_TYPE_RAW) {
            while (raw_index >= raw_signals_.size()) {
                raw_signals_.emplace_back();
            }
            InsertTypedChannel(&raw_signals_[raw_index], channel_data,
                               audio_data.microphone_config());
            ++raw_index;
        } else if (channel_data.channel_type() == proto::CHANNEL_TYPE_ASR) {
            InsertTypedChannel(&asr_signal_, channel_data,
                               audio_data.microphone_config());
            has_asr_channel_ = true;
        }
    }
}

void AudioInfo::InsertTypedChannel(
    std::deque<double>* dest, const proto::ChannelData& channel_data,
    const proto::MicrophoneConfig& microphone_config) {
    const int width = microphone_config.sample_width() > 0
                          ? microphone_config.sample_width()
                          : 2;
    const std::string& data = channel_data.data();
    for (std::size_t i = 0; i + static_cast<std::size_t>(width) <= data.size();
         i += static_cast<std::size_t>(width)) {
        const auto lo = static_cast<uint8_t>(data[i]);
        const auto hi = static_cast<uint8_t>(data[i + 1]);
        const int16_t sample =
            static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
        dest->push_back(static_cast<double>(sample));
    }
    const double sample_rate = microphone_config.sample_rate() > 0.0F
                                   ? microphone_config.sample_rate()
                                   : 48000.0;
    const std::size_t max_signal_length = static_cast<std::size_t>(
        cache_signal_time_sec_ * sample_rate);
    while (dest->size() > max_signal_length) {
        dest->pop_front();
    }
}

std::vector<std::vector<double>> AudioInfo::GetSignals(
    const int signal_length) const {
    std::vector<std::vector<double>> signals;
    signals.reserve(raw_signals_.size());
    for (const auto& channel : raw_signals_) {
        const int start_index =
            std::max(0, static_cast<int>(channel.size()) - signal_length);
        signals.emplace_back(channel.begin() + start_index, channel.end());
    }
    return signals;
}

std::vector<double> AudioInfo::GetAsrSignal(const int signal_length) const {
    const std::deque<double>* src = nullptr;
    if (has_asr_channel_ && !asr_signal_.empty()) {
        src = &asr_signal_;
    } else if (!raw_signals_.empty() && !raw_signals_.front().empty()) {
        src = &raw_signals_.front();
    }
    if (src == nullptr) {
        return {};
    }
    const int start_index =
        std::max(0, static_cast<int>(src->size()) - signal_length);
    return {src->begin() + start_index, src->end()};
}

}  // namespace audio
}  // namespace autonomy
