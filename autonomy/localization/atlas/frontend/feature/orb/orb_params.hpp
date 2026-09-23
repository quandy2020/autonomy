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
 *
 * Parameters mirror ORB-SLAM3 ORBextractor settings.
 */

/**
 * @file orb_params.hpp
 * @brief Configurable ORB extractor parameters (ORB-SLAM3 ORBextractor settings).
 *
 * Held by Tracker::Options::orb and passed to the OrbExtractor constructor.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_PARAMS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_PARAMS_HPP_

namespace autonomy {
namespace localization {
namespace atlas {
namespace feature {

/**
 * @struct autonomy::localization::atlas::feature::OrbParams
 * @brief ORB feature extraction hyperparameters (aligned with ORB-SLAM3 `ORBextractor`).
 *
 * Held by `Tracker::Options::orb` and passed when constructing `OrbExtractor`.
 * When texture is poor, the extractor lowers from `initial_fast_threshold` to
 * `minimum_fast_threshold`.
 *
 * @par Usage
 * @code{.cpp}
 * feature::OrbParams p;
 * p.num_features = 1200;
 * p.scale_factor = 1.2f;
 * p.num_levels = 8;
 * feature::OrbExtractor extractor(p);
 * @endcode
 */
struct OrbParams {
    int num_features = 1000;  ///< Target feature count over the image (per-level share)
    float scale_factor = 1.2f;  ///< Adjacent pyramid scale ratio \(s>1\)
    int num_levels = 8;  ///< Pyramid levels (including original)
    int initial_fast_threshold = 20;  ///< Initial FAST response threshold
    int minimum_fast_threshold = 7;  ///< Lower bound when features are scarce
};

}  // namespace feature
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_PARAMS_HPP_
