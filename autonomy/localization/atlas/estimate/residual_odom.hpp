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

#include "autonomy/localization/atlas/type.hpp"

#include <deque>
#include <mutex>
#include <vector>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
}

namespace estimate {

struct OdomDeltaResidual {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp = 0.0;
    Mat44_t T_delta = Mat44_t::Identity();  // body_k ← body_{k-1} or relative
    double weight = 1.0;
};

struct OdomFactorBatch {
    std::vector<OdomDeltaResidual> deltas;
    [[nodiscard]] bool empty() const { return deltas.empty(); }
};

class IOdomResidualSource {
public:
    virtual ~IOdomResidualSource() = default;
    virtual OdomFactorBatch Pull(double t0, double t1) = 0;
};

class BufferedOdomResidualSource final : public IOdomResidualSource {
public:
    explicit BufferedOdomResidualSource(std::size_t max_batches = 128)
        : max_batches_(max_batches) {}

    void Push(OdomDeltaResidual residual) {
        std::lock_guard<std::mutex> lock(mtx_);
        buffer_.push_back(std::move(residual));
        while (buffer_.size() > max_batches_) {
            buffer_.pop_front();
        }
    }

    OdomFactorBatch Pull(double t0, double t1) override {
        OdomFactorBatch out;
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto& item : buffer_) {
            if (item.timestamp < t0 || item.timestamp > t1) {
                continue;
            }
            out.deltas.push_back(item);
        }
        return out;
    }

private:
    std::size_t max_batches_;
    mutable std::mutex mtx_;
    std::deque<OdomDeltaResidual> buffer_;
};

//! Apply latest odom delta as a soft prior on keyframe pose (T_cw).
int ApplyOdomDeltaRefine(data::keyframe* keyfrm, const OdomFactorBatch& batch);

}  // namespace estimate
}  // namespace autonomy::localization::atlas
