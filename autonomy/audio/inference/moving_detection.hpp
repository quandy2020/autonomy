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

#ifndef AUTONOMY_AUDIO_INFERENCE_MOVING_DETECTION_HPP_
#define AUTONOMY_AUDIO_INFERENCE_MOVING_DETECTION_HPP_

#include "autonomy/audio/proto/audio_common.pb.h"

#include <complex>
#include <deque>
#include <vector>

namespace autonomy {
namespace audio {

class MovingDetection {
public:
    MovingDetection() = default;

    std::vector<std::complex<double>> fft1d(
        const std::vector<double>& signals);

    proto::MovingResult Detect(
        const std::vector<std::vector<double>>& signals);

    proto::MovingResult DetectSingleChannel(std::size_t channel_index,
                                            const std::vector<double>& signal);

private:
    class SignalStat {
    public:
        SignalStat(double power, int top_frequency)
            : power_(power), top_frequency_(top_frequency) {}
        double power() const { return power_; }
        int top_frequency() const { return top_frequency_; }

    private:
        double power_;
        int top_frequency_;
    };

    SignalStat GetSignalStat(
        const std::vector<std::complex<double>>& fft_results,
        int start_frequency);

    proto::MovingResult AnalyzePower(const std::deque<SignalStat>& signal_stats);

    proto::MovingResult AnalyzeTopFrequence(
        const std::deque<SignalStat>& signal_stats);

    std::vector<std::deque<SignalStat>> signal_stats_;
};

}  // namespace audio
}  // namespace autonomy

#endif  // AUTONOMY_AUDIO_INFERENCE_MOVING_DETECTION_HPP_
