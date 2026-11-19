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

#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include <fbow/fbow.h>
#include <opencv2/core/core.hpp>

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace ORB_SLAM3
{

// The migrated ORB-SLAM3 headers were written against the global
// using-directive that the bundled DBoW2 vocabulary header introduced.
// Scope the same name lookup to this namespace so these sources keep
// compiling without leaking std into every translation unit.
using namespace std;

using WordId = std::uint32_t;
using WordValue = double;
using NodeId = std::uint32_t;

// Bag-of-words vector keyed by FBoW leaf id.
using BowVector = std::map<WordId, WordValue>;
// Feature groups at the requested tree level: node id -> descriptor indices.
using FeatureVector = std::map<NodeId, std::vector<unsigned int>>;

/**
 * @brief ORB vocabulary backed by FBoW.
 *
 * This replaces the DBoW2 TemplatedVocabulary that ORB-SLAM3 shipped with.
 * The class keeps the small vocabulary surface used by tracking, matching and
 * place recognition (load, transform, score) while storing a binary .fbow
 * vocabulary produced by the FBoW tools. The vocabulary file is shared with
 * autonomy/localization/atlas, which already uses FBoW for the same ORB
 * descriptors.
 */
class ORBVocabulary
{
public:
    ORBVocabulary() = default;
    ORBVocabulary(const ORBVocabulary&) = delete;
    ORBVocabulary& operator=(const ORBVocabulary&) = delete;
    ~ORBVocabulary() = default;

    /** @brief Loads a binary .fbow vocabulary. @return false on failure. */
    bool Load(const std::string& path);

    bool IsValid() const;
    bool empty() const;

    /** @brief Highest word id observed plus one. */
    unsigned int size() const;

    /**
     * @brief Quantizes ORB descriptors into a bag of words.
     * @param descriptors N x 32 CV_8U ORB descriptor matrix.
     * @param bow Output word weights, L1 normalized like DBoW2.
     * @param features Output descriptors grouped by tree node.
     * @param levels_up Tree levels counted from the leaves (DBoW2 semantics).
     */
    void Transform(const cv::Mat& descriptors, BowVector* bow,
                   FeatureVector* features, int levels_up = 4);
    /** @brief Overload accepting one descriptor row per keypoint. */
    void Transform(const std::vector<cv::Mat>& descriptors, BowVector* bow,
                   FeatureVector* features, int levels_up = 4);

    /**
     * @brief L1 similarity in [0, 1] with the DBoW2 scoring definition.
     *
     * ORB-SLAM3 thresholds (loop closing, relocalization) were tuned against
     * this metric, so FBoW's built-in L2 score is deliberately not used.
     */
    WordValue Score(const BowVector& lhs, const BowVector& rhs) const;

    void clear();

private:
    /** FBoW levels are counted from the root of the six level vocabulary. */
    static constexpr int kVocabularyLevels = 6;
    static int FeatureLevelFor(int levels_up);

    fbow::Vocabulary vocabulary_;
    mutable std::mutex mutex_;
    unsigned int max_word_id_ = 0;
};

} //namespace ORB_SLAM3

#endif // ORBVOCABULARY_H

