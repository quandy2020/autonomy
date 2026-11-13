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

/**
 * @file value_tier.cpp
 * @brief P0–P3 upload policy; P2/P3 Bernoulli sample seeded by episode_id.
 */

#include "autonomy/system/recorder/value_tier.hpp"

#include <cstdint>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

uint64_t Mix(const std::string& s) {
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}

bool Keep(const std::string& key, double rate) {
    if (rate >= 1.0) {
        return true;
    }
    if (rate <= 0.0) {
        return false;
    }
    const double u =
        static_cast<double>(Mix(key) & 0xffffff) / static_cast<double>(0xffffff);
    return u < rate;
}

}  // namespace

UploadDecision DecideUpload(proto::Priority priority,
                            const std::string& episode_id,
                            const std::string& scene, double p2_rate,
                            double p3_rate) {
    UploadDecision d;
    d.priority = priority;
    d.expert_demo = false;
    const std::string key = episode_id + "|" + scene;
    switch (priority) {
        case proto::PRIORITY_P0:
            d.upload = true;
            d.policy = "immediate";
            break;
        case proto::PRIORITY_P1:
            d.upload = true;
            d.policy = "upload_not_expert";
            break;
        case proto::PRIORITY_P2:
            d.sampled = true;
            d.upload = Keep(key, p2_rate > 0 ? p2_rate : 0.5);
            d.policy = d.upload ? "p2_sample_keep" : "p2_sample_drop";
            break;
        case proto::PRIORITY_P3:
        default:
            d.sampled = true;
            d.upload = Keep(key, p3_rate > 0 ? p3_rate : 0.1);
            d.policy = d.upload ? "p3_stratified_keep" : "p3_stratified_drop";
            break;
    }
    return d;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
