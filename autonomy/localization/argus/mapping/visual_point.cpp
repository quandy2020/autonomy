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

#include "argus/map/visual_point.hpp"

#include <limits>

namespace argus {
namespace map {

VisualPoint::VisualPoint(const Eigen::Vector3d& pos)
    : pos_(pos),
      previous_normal_(Eigen::Vector3d::Zero()),
      normal_(Eigen::Vector3d::Zero()),
      is_converged_(false),
      is_normal_initialized_(false),
      has_ref_patch_(false) {}

VisualPoint::~VisualPoint() {
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it) {
        delete (*it);
    }
    obs_.clear();
    ref_patch = nullptr;
}

void VisualPoint::AddFrameRef(Feature* ftr) {
    obs_.push_front(ftr);
}

void VisualPoint::DeleteFeatureRef(Feature* ftr) {
    if (ref_patch == ftr) {
        ref_patch = nullptr;
        has_ref_patch_ = false;
    }
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it) {
        if ((*it) == ftr) {
            delete (*it);
            obs_.erase(it);
            return;
        }
    }
}

bool VisualPoint::GetCloseViewObs(const Eigen::Vector3d& framepos,
                                  Feature*& ftr,
                                  const Eigen::Vector2d& /*cur_px*/) const {
    if (obs_.empty()) {
        return false;
    }

    Eigen::Vector3d obs_dir(framepos - pos_);
    obs_dir.normalize();
    auto min_it = obs_.begin();
    double min_cos_angle = 0;
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it) {
        Eigen::Vector3d dir((*it)->T_f_w_.inverse().translation() - pos_);
        dir.normalize();
        const double cos_angle = obs_dir.dot(dir);
        if (cos_angle > min_cos_angle) {
            min_cos_angle = cos_angle;
            min_it = it;
        }
    }
    ftr = *min_it;
    if (min_cos_angle < 0.5) {
        return false;
    }
    return true;
}

void VisualPoint::FindMinScoreFeature(const Eigen::Vector3d& /*framepos*/,
                                      Feature*& ftr) const {
    auto min_it = obs_.begin();
    float min_score = std::numeric_limits<float>::max();
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it) {
        if ((*it)->score_ < min_score) {
            min_score = (*it)->score_;
            min_it = it;
        }
    }
    ftr = *min_it;
}

void VisualPoint::DeleteNonRefPatchFeatures() {
    for (auto it = obs_.begin(); it != obs_.end();) {
        if (*it != ref_patch) {
            delete *it;
            it = obs_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace map
}  // namespace argus
