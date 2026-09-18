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

#pragma once

//! Gyro-only multi-stage filter (Lightning IMUFilter): spike → median → MA → rate limit.
//! Acc left unchanged; used before BuildImuPoses / PredictImu.

#include "autonomy/localization/atlas/sensor/types.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace frontend {
namespace lio {

class ImuFilter {
public:
    struct Config {
        int median_window_size = 5;     // odd ≥3
        int moving_avg_window = 3;
        double rate_limit = 3.0;        // rad/s²
        double spike_threshold = 3.0;   // std multiples
        bool enable_adaptive = true;
    };

    ImuFilter() { prev_filtered_.timestamp = -1.0; }

    void set_config(Config c) {
        if (c.median_window_size >= 3 && (c.median_window_size % 2) == 1) {
            config_.median_window_size = c.median_window_size;
        }
        if (c.moving_avg_window >= 1) {
            config_.moving_avg_window = c.moving_avg_window;
        }
        config_.rate_limit = std::abs(c.rate_limit);
        config_.spike_threshold = std::abs(c.spike_threshold);
        config_.enable_adaptive = c.enable_adaptive;
    }
    [[nodiscard]] const Config& config() const { return config_; }

    void SetMedianWindowSize(int size) {
        if (size >= 3 && (size % 2) == 1) {
            config_.median_window_size = size;
        }
    }
    void SetRateLimit(double limit) { config_.rate_limit = std::abs(limit); }
    void SetSpikeThreshold(double th) {
        config_.spike_threshold = std::abs(th);
    }

    void Reset() {
        gyro_x_history_.clear();
        gyro_y_history_.clear();
        gyro_z_history_.clear();
        prev_filtered_.timestamp = -1.0;
        sample_count_ = 0;
        for (int i = 0; i < 3; ++i) {
            gyro_mean_[i] = 0.0;
            gyro_std_[i] = 0.1;
        }
    }

    sensor::ImuSample Filter(const sensor::ImuSample& raw) {
        sensor::ImuSample filtered = raw;
        UpdateBuffer(raw);
        filtered.gyro.x() =
            ProcessAxis(raw.gyro.x(), gyro_x_history_, 0);
        filtered.gyro.y() =
            ProcessAxis(raw.gyro.y(), gyro_y_history_, 1);
        filtered.gyro.z() =
            ProcessAxis(raw.gyro.z(), gyro_z_history_, 2);

        if (prev_filtered_.timestamp > 0.0) {
            const double dt = raw.timestamp - prev_filtered_.timestamp;
            if (dt > 0.0 && dt < 0.1) {
                filtered.gyro.x() = RateLimit(filtered.gyro.x(),
                                              prev_filtered_.gyro.x(), dt);
                filtered.gyro.y() = RateLimit(filtered.gyro.y(),
                                              prev_filtered_.gyro.y(), dt);
                filtered.gyro.z() = RateLimit(filtered.gyro.z(),
                                              prev_filtered_.gyro.z(), dt);
            }
        }
        UpdateStatistics(filtered);
        prev_filtered_ = filtered;
        return filtered;
    }

private:
    double ProcessAxis(double raw_value, std::deque<double>& history,
                       int axis_idx) {
        double filtered = raw_value;
        if (static_cast<int>(history.size()) >= config_.median_window_size &&
            sample_count_ > 100) {
            filtered = DetectAndRemoveSpike(raw_value, history, axis_idx);
        }
        filtered = MedianFilter(filtered, history);
        filtered = MovingAverage(filtered, history);
        return filtered;
    }

    void UpdateBuffer(const sensor::ImuSample& data) {
        gyro_x_history_.push_back(data.gyro.x());
        gyro_y_history_.push_back(data.gyro.y());
        gyro_z_history_.push_back(data.gyro.z());
        const int max_history = std::max(
            {config_.median_window_size, config_.moving_avg_window, 10});
        while (static_cast<int>(gyro_x_history_.size()) > max_history) {
            gyro_x_history_.pop_front();
            gyro_y_history_.pop_front();
            gyro_z_history_.pop_front();
        }
    }

    double DetectAndRemoveSpike(double value, std::deque<double>& history,
                                int axis_idx) {
        if (static_cast<int>(history.size()) < config_.median_window_size) {
            return value;
        }
        std::vector<double> window(history.end() - config_.median_window_size,
                                   history.end());
        std::nth_element(window.begin(),
                         window.begin() + static_cast<int>(window.size()) / 2,
                         window.end());
        const double median = window[window.size() / 2];
        const double diff = std::abs(value - median);
        double threshold = config_.spike_threshold * gyro_std_[axis_idx];
        if (config_.enable_adaptive && gyro_std_[axis_idx] > 0.0) {
            threshold = std::max(threshold, config_.spike_threshold * 0.5);
        }
        if (diff > threshold) {
            AINFO_EVERY(50) << "ImuFilter spike: " << diff << " > " << threshold;
            return median;
        }
        return value;
    }

    double MedianFilter(double value, std::deque<double>& history) {
        if (static_cast<int>(history.size()) < config_.median_window_size) {
            return value;
        }
        std::vector<double> window(history.end() - config_.median_window_size,
                                   history.end());
        std::nth_element(window.begin(),
                         window.begin() + static_cast<int>(window.size()) / 2,
                         window.end());
        return window[window.size() / 2];
    }

    double MovingAverage(double value, std::deque<double>& history) {
        if (static_cast<int>(history.size()) < config_.moving_avg_window) {
            return value;
        }
        double sum = 0.0;
        auto it = history.end() - config_.moving_avg_window;
        for (; it != history.end(); ++it) {
            sum += *it;
        }
        return sum / static_cast<double>(config_.moving_avg_window);
    }

    double RateLimit(double current, double previous, double dt) const {
        const double max_change = config_.rate_limit * dt;
        const double diff = current - previous;
        if (std::abs(diff) > max_change) {
            return previous + (diff > 0.0 ? max_change : -max_change);
        }
        return current;
    }

    void UpdateStatistics(const sensor::ImuSample& data) {
        constexpr double kAlpha = 0.01;
        if (sample_count_ == 0) {
            gyro_mean_[0] = data.gyro[0];
            gyro_mean_[1] = data.gyro[1];
            gyro_mean_[2] = data.gyro[2];
            gyro_std_[0] = gyro_std_[1] = gyro_std_[2] = 0.1;
        } else {
            for (int i = 0; i < 3; ++i) {
                gyro_mean_[i] =
                    (1.0 - kAlpha) * gyro_mean_[i] + kAlpha * data.gyro[i];
                gyro_std_[i] = (1.0 - kAlpha) * gyro_std_[i] +
                               kAlpha * std::abs(data.gyro[i] - gyro_mean_[i]);
            }
        }
        ++sample_count_;
    }

    Config config_;
    std::deque<double> gyro_x_history_;
    std::deque<double> gyro_y_history_;
    std::deque<double> gyro_z_history_;
    sensor::ImuSample prev_filtered_;
    double gyro_mean_[3] = {0.0, 0.0, 0.0};
    double gyro_std_[3] = {0.1, 0.1, 0.1};
    int sample_count_ = 0;
};

}  // namespace lio
}  // namespace frontend
}  // namespace autonomy::localization::atlas
