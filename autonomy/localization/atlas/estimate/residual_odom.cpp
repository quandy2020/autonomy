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

#include "autonomy/localization/atlas/estimate/residual_odom.hpp"

#include "autonomy/localization/atlas/data/keyframe.hpp"

#include <algorithm>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace estimate {

int ApplyOdomDeltaRefine(data::keyframe* keyfrm, const OdomFactorBatch& batch) {
    if (!keyfrm || batch.empty()) {
        return 0;
    }
    // Use the newest delta in window: T_cw' ≈ T_delta^{-1} * T_cw (body motion).
    const OdomDeltaResidual& r = batch.deltas.back();
    const Mat44_t T_cw = keyfrm->get_pose_cw();
    const Mat44_t T_wc = T_cw.inverse();
    const Mat44_t T_wc_new = T_wc * r.T_delta;
    // Blend toward predicted pose (weight as lerp on translation / slerp skip).
    const double a = std::min(1.0, std::max(0.0, r.weight));
    Mat44_t blended = T_wc;
    blended.block<3, 1>(0, 3) =
        (1.0 - a) * T_wc.block<3, 1>(0, 3) + a * T_wc_new.block<3, 1>(0, 3);
    if (a > 0.5) {
        blended.block<3, 3>(0, 0) = T_wc_new.block<3, 3>(0, 0);
    }
    keyfrm->set_pose_cw(blended.inverse());
    AINFO << "ApplyOdomDeltaRefine: applied " << batch.deltas.size()
          << " odom delta(s)";
    return static_cast<int>(batch.deltas.size());
}

}  // namespace estimate
}  // namespace autonomy::localization::atlas
