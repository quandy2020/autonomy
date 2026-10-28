// Copyright 2026 The Openbot Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_FEATURE_VOCABULARY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_FEATURE_VOCABULARY_HPP_

#include <fbow/fbow.h>
#include <opencv2/core/mat.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {

using WordId = std::uint32_t;
using WordValue = double;
using NodeId = std::uint32_t;
using BowVector = std::map<WordId, WordValue>;
using FeatureVector = std::map<NodeId, std::vector<unsigned int>>;

// Adapts the project-wide FBoW vocabulary to the subset of the historic
// ORB vocabulary API used by tracking, matching, and loop closing.
class Vocabulary
{
public:
    Vocabulary() = default;

    bool LoadFromFile(const std::string& filename);
    bool IsValid() const;

    void Transform(const cv::Mat& descriptors, BowVector* bow,
                   FeatureVector* features);
    void Transform(const std::vector<cv::Mat>& descriptors, BowVector* bow,
                   FeatureVector* features);

    // Uses the L1 score expected by the original ORB-SLAM3 thresholds instead
    // of FBoW's built-in L2 score.
    static double Score(const BowVector& lhs, const BowVector& rhs);

private:
    // FBoW levels are absolute from the root. The project vocabulary has six
    // levels; level two corresponds to the old "four levels up" feature nodes.
    static constexpr int kFeatureLevel = 2;

    fbow::Vocabulary vocabulary_;
    mutable std::mutex mutex_;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_FEATURE_VOCABULARY_HPP_
