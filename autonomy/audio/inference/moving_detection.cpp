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

#include "autonomy/audio/inference/moving_detection.hpp"

#include "autonomy/audio/inference/fft.hpp"

#include <cmath>

namespace autonomy {
namespace audio {

proto::MovingResult MovingDetection::Detect(
    const std::vector<std::vector<double>>& signals) {
    int approaching_count = 0;
    int departing_count = 0;
    for (std::size_t i = 0; i < signals.size(); ++i) {
        while (signal_stats_.size() <= i) {
            signal_stats_.emplace_back();
        }
        const proto::MovingResult result =
            DetectSingleChannel(i, signals[i]);
        if (result == proto::MOVING_RESULT_APPROACHING) {
            ++approaching_count;
        } else if (result == proto::MOVING_RESULT_DEPARTING) {
            ++departing_count;
        }
    }
    if (approaching_count > departing_count) {
        return proto::MOVING_RESULT_APPROACHING;
    }
    if (approaching_count < departing_count) {
        return proto::MOVING_RESULT_DEPARTING;
    }
    return proto::MOVING_RESULT_UNKNOWN;
}

proto::MovingResult MovingDetection::DetectSingleChannel(
    const std::size_t channel_index, const std::vector<double>& signals) {
    static constexpr int kStartFrequency = 3;
    static constexpr int kFrameNumStored = 10;
    const std::vector<std::complex<double>> fft_results = fft1d(signals);
    const SignalStat signal_stat =
        GetSignalStat(fft_results, kStartFrequency);
    signal_stats_[channel_index].push_back(signal_stat);
    while (static_cast<int>(signal_stats_[channel_index].size()) >
           kFrameNumStored) {
        signal_stats_[channel_index].pop_front();
    }
    const proto::MovingResult power_result =
        AnalyzePower(signal_stats_[channel_index]);
    if (power_result != proto::MOVING_RESULT_UNKNOWN) {
        return power_result;
    }
    return AnalyzeTopFrequence(signal_stats_[channel_index]);
}

proto::MovingResult MovingDetection::AnalyzePower(
    const std::deque<SignalStat>& signal_stats) {
    const int n = static_cast<int>(signal_stats.size());
    if (n < 3) {
        return proto::MOVING_RESULT_UNKNOWN;
    }
    const double first = signal_stats[n - 3].power();
    const double second = signal_stats[n - 2].power();
    const double third = signal_stats[n - 1].power();
    if (first < second && second < third) {
        return proto::MOVING_RESULT_APPROACHING;
    }
    if (first > second && second > third) {
        return proto::MOVING_RESULT_DEPARTING;
    }
    return proto::MOVING_RESULT_UNKNOWN;
}

proto::MovingResult MovingDetection::AnalyzeTopFrequence(
    const std::deque<SignalStat>& signal_stats) {
    const int n = static_cast<int>(signal_stats.size());
    if (n < 3) {
        return proto::MOVING_RESULT_UNKNOWN;
    }
    const int first = signal_stats[n - 3].top_frequency();
    const int second = signal_stats[n - 2].top_frequency();
    const int third = signal_stats[n - 1].top_frequency();
    if (first < second && second < third) {
        return proto::MOVING_RESULT_APPROACHING;
    }
    if (first > second && second > third) {
        return proto::MOVING_RESULT_DEPARTING;
    }
    return proto::MOVING_RESULT_UNKNOWN;
}

std::vector<std::complex<double>> MovingDetection::fft1d(
    const std::vector<double>& signal) {
    return Fft1d(signal);
}

MovingDetection::SignalStat MovingDetection::GetSignalStat(
    const std::vector<std::complex<double>>& fft_results,
    const int start_frequency) {
    double total_power = 0.0;
    int top_frequency = -1;
    double max_power = -1.0;
    for (int i = start_frequency; i < static_cast<int>(fft_results.size());
         ++i) {
        const double power = std::abs(fft_results[i]);
        total_power += power;
        if (power > max_power) {
            max_power = power;
            top_frequency = i;
        }
    }
    return {total_power, top_frequency};
}

}  // namespace audio
}  // namespace autonomy
