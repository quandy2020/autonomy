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

/**
 * @file orb_vocabulary.cpp
 * @brief OrbVocabulary implementation: FBoW load, Transform, and Score.
 */

#include "autonomy/localization/atlas/frontend/feature/orb/orb_vocabulary.hpp"

#include <iostream>

namespace autonomy {
namespace localization {
namespace atlas {
namespace feature {

bool OrbVocabulary::Load(const std::string& path) {
    auto vocabulary = std::make_unique<fbow::Vocabulary>();
    try {
        vocabulary->readFromFile(path);
    } catch (const std::exception& ex) {
        std::cerr << "[atlas] FBoW load failed: " << path << " (" << ex.what()
                  << ")\n";
        return false;
    }
    if (!vocabulary->isValid()) {
        std::cerr << "[atlas] FBoW vocabulary invalid: " << path << "\n";
        return false;
    }
    vocabulary_ = std::move(vocabulary);
    path_ = path;
    return true;
}

bool OrbVocabulary::is_valid() const {
    return vocabulary_ != nullptr && vocabulary_->isValid();
}

void OrbVocabulary::Transform(const cv::Mat& descriptors, fbow::BoWVector* bow,
                              fbow::BoWFeatVector* feat, int level) const {
    if (bow == nullptr || feat == nullptr || !is_valid() ||
        descriptors.empty()) {
        if (bow != nullptr) {
            bow->clear();
        }
        if (feat != nullptr) {
            feat->clear();
        }
        return;
    }
    vocabulary_->transform(descriptors, level, *bow, *feat);
}

double OrbVocabulary::Score(const fbow::BoWVector& a,
                            const fbow::BoWVector& b) const {
    return fbow::BoWVector::score(a, b);
}

}  // namespace feature
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
