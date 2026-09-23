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
 * ORB vocabulary via FBoW (Fast Bag of Words), replacing DBoW2.
 */

/**
 * @file orb_vocabulary.hpp
 * @brief Thin ORB bag-of-words wrapper on FBoW (replaces DBoW2).
 *
 * Used for Frame/KeyFrame BoW vectors, match acceleration, and keyframe DB retrieval.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_VOCABULARY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_VOCABULARY_HPP_

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

#include <fbow/bow_feat_vector.h>
#include <fbow/bow_vector.h>
#include <fbow/vocabulary.h>

namespace autonomy {
namespace localization {
namespace atlas {
namespace feature {

/**
 * @class autonomy::localization::atlas::feature::OrbVocabulary
 * @brief Thin wrapper around @c fbow::Vocabulary for ORB descriptors.
 *
 * Typical usage: `Load(path)` → `Transform(descriptors, &bow, &feat)` →
 * `Score` to compare BoW vectors; called by Frame::ComputeBoW / KeyFrameDatabase.
 *
 * @note Load during init; Transform/Score have no writable internal state and may
 *       be shared read-only across threads (after vocabulary_ is fully loaded).
 */
class OrbVocabulary {
public:
    //! BoWFeatVector tree depth (ORB-SLAM3 commonly uses level 4 with DBoW2).
    static constexpr int kDefaultFeatLevel = 4;

    OrbVocabulary() = default;

    /**
     * @brief Load an FBoW vocabulary from file.
     * @param path Vocabulary file path.
     * @return true if read succeeded and vocabulary is valid.
     */
    bool Load(const std::string& path);

    /**
     * @brief Whether the vocabulary loaded successfully.
     * @return true if valid.
     */
    bool is_valid() const;

    /**
     * @brief Transform ORB descriptors into BoW and Feat vectors.
     * @param descriptors N×32 descriptor matrix.
     * @param[out] bow BoW histogram vector.
     * @param[out] feat Feature indices organized by tree nodes.
     * @param level Feat vector level; default kDefaultFeatLevel.
     */
    void Transform(const cv::Mat& descriptors, fbow::BoWVector* bow,
                   fbow::BoWFeatVector* feat,
                   int level = kDefaultFeatLevel) const;

    /**
     * @brief Similarity score between two BoW vectors.
     * @param a,b Input BoW vectors.
     * @return Score (higher = more similar; metric defined by FBoW).
     */
    double Score(const fbow::BoWVector& a,
                               const fbow::BoWVector& b) const;

    /**
     * @brief Path of the loaded vocabulary file.
     */
    const std::string& path() const { return path_; }

private:
    std::unique_ptr<fbow::Vocabulary> vocabulary_;  ///< FBoW vocabulary
    std::string path_;                               ///< Load path
};

}  // namespace feature
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_VOCABULARY_HPP_
