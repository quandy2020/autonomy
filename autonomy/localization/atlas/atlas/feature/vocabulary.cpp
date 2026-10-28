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

#include "atlas/feature/vocabulary.hpp"

#include "autolink/common/log.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace autonomy::localization::atlas {

bool Vocabulary::LoadFromFile(const std::string& filename) {
    const std::lock_guard<std::mutex> lock(mutex_);
    try {
        vocabulary_.readFromFile(filename);
    } catch (const std::exception& error) {
        AERROR << "Failed to load FBoW vocabulary " << filename << ": "
               << error.what();
        return false;
    } catch (...) {
        AERROR << "Failed to load FBoW vocabulary " << filename
               << ": unknown exception";
        return false;
    }
    if (!vocabulary_.isValid()) {
        AERROR << "Invalid FBoW vocabulary: " << filename;
        return false;
    }
    AINFO << "Loaded FBoW vocabulary: " << filename;
    return true;
}

bool Vocabulary::IsValid() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return vocabulary_.isValid();
}

void Vocabulary::Transform(const cv::Mat& descriptors, BowVector* bow,
                           FeatureVector* features) {
    if (bow == nullptr || features == nullptr) {
        AERROR << "FBoW transform requires non-null output containers";
        return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    bow->clear();
    features->clear();
    if (!vocabulary_.isValid() || descriptors.empty()) {
        return;
    }
    if (descriptors.type() != CV_8U || descriptors.cols != 32) {
        AERROR << "ORB descriptors must be an N x 32 CV_8U matrix";
        return;
    }

    fbow::BoWVector fbow_bow;
    fbow::BoWFeatVector fbow_features;
    vocabulary_.transform(descriptors, kFeatureLevel, fbow_bow, fbow_features);
    for (const auto& [word_id, value] : fbow_bow) {
        if (!std::isfinite(value)) {
            AERROR << "FBoW produced a non-finite word weight";
            bow->clear();
            features->clear();
            return;
        }
        bow->emplace(static_cast<WordId>(word_id),
                     static_cast<WordValue>(value));
    }
    for (const auto& [node_id, indices] : fbow_features) {
        auto& output = (*features)[static_cast<NodeId>(node_id)];
        output.reserve(indices.size());
        for (const auto index : indices) {
            output.push_back(static_cast<unsigned int>(index));
        }
    }
}

void Vocabulary::Transform(const std::vector<cv::Mat>& descriptors,
                           BowVector* bow, FeatureVector* features) {
    if (descriptors.empty()) {
        if (bow != nullptr) {
            bow->clear();
        }
        if (features != nullptr) {
            features->clear();
        }
        return;
    }
    cv::Mat matrix(static_cast<int>(descriptors.size()), 32, CV_8U);
    for (std::size_t index = 0; index < descriptors.size(); ++index) {
        const cv::Mat& descriptor = descriptors[index];
        if (descriptor.type() != CV_8U || descriptor.total() != 32) {
            AERROR << "ORB descriptor " << index << " is not 32-byte CV_8U";
            if (bow != nullptr) {
                bow->clear();
            }
            if (features != nullptr) {
                features->clear();
            }
            return;
        }
        descriptor.reshape(1, 1).copyTo(matrix.row(static_cast<int>(index)));
    }
    Transform(matrix, bow, features);
}

double Vocabulary::Score(const BowVector& lhs, const BowVector& rhs) {
    auto lhs_it = lhs.begin();
    auto rhs_it = rhs.begin();
    double l1_distance = 0.0;
    while (lhs_it != lhs.end() || rhs_it != rhs.end()) {
        if (rhs_it == rhs.end() ||
            (lhs_it != lhs.end() && lhs_it->first < rhs_it->first)) {
            if (!std::isfinite(lhs_it->second)) {
                return 0.0;
            }
            l1_distance += std::abs(lhs_it->second);
            ++lhs_it;
        } else if (lhs_it == lhs.end() || rhs_it->first < lhs_it->first) {
            if (!std::isfinite(rhs_it->second)) {
                return 0.0;
            }
            l1_distance += std::abs(rhs_it->second);
            ++rhs_it;
        } else {
            if (!std::isfinite(lhs_it->second) ||
                !std::isfinite(rhs_it->second)) {
                return 0.0;
            }
            l1_distance += std::abs(lhs_it->second - rhs_it->second);
            ++lhs_it;
            ++rhs_it;
        }
    }
    return std::clamp(1.0 - 0.5 * l1_distance, 0.0, 1.0);
}

}  // namespace autonomy::localization::atlas
