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

#include "ORBVocabulary.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>

namespace ORB_SLAM3
{

namespace
{

constexpr int kOrbDescriptorBytes = 32;

bool EndsWith(const std::string& text, const std::string& suffix)
{
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace

int ORBVocabulary::FeatureLevelFor(int levels_up)
{
    // DBoW2 counts levels from the leaves, FBoW from the root.
    const int level = kVocabularyLevels - levels_up;
    return std::clamp(level, 0, kVocabularyLevels - 1);
}

bool ORBVocabulary::Load(const std::string& path)
{
    if (EndsWith(path, ".txt"))
    {
        std::cerr << "ORBVocabulary: " << path
                  << " is a DBoW2 text vocabulary. FBoW requires a binary "
                     ".fbow file; generate one with the FBoW tools."
                  << std::endl;
        return false;
    }

    const std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        vocabulary_.readFromFile(path);
    }
    catch (const std::exception& error)
    {
        std::cerr << "ORBVocabulary: failed to load " << path << ": "
                  << error.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "ORBVocabulary: failed to load " << path
                  << ": unknown exception" << std::endl;
        return false;
    }

    if (!vocabulary_.isValid())
    {
        std::cerr << "ORBVocabulary: invalid vocabulary " << path << std::endl;
        return false;
    }

    max_word_id_ = 0;
    std::cout << "ORBVocabulary: loaded FBoW vocabulary " << path
              << " (descriptor " << vocabulary_.getDescName() << ", "
              << vocabulary_.getDescSize() << " bytes)"
              << std::endl;
    return true;
}

bool ORBVocabulary::IsValid() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return vocabulary_.isValid();
}

bool ORBVocabulary::empty() const
{
    return !IsValid();
}

unsigned int ORBVocabulary::size() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return max_word_id_ == 0 ? 0 : max_word_id_ + 1;
}

void ORBVocabulary::Transform(const cv::Mat& descriptors, BowVector* bow,
                              FeatureVector* features, int levels_up)
{
    if (bow == nullptr || features == nullptr)
    {
        std::cerr << "ORBVocabulary: transform requires non-null outputs"
                  << std::endl;
        return;
    }

    bow->clear();
    features->clear();

    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (!vocabulary_.isValid() || descriptors.empty())
        {
            return;
        }
        if (descriptors.type() != CV_8U || descriptors.cols != kOrbDescriptorBytes)
        {
            std::cerr << "ORBVocabulary: ORB descriptors must be N x 32 CV_8U"
                      << std::endl;
            return;
        }
        if (vocabulary_.getDescSize() != kOrbDescriptorBytes)
        {
            std::cerr << "ORBVocabulary: vocabulary expects "
                      << vocabulary_.getDescSize()
                      << "-byte descriptors, ORB supplies 32" << std::endl;
            return;
        }
    }

    fbow::BoWVector fbow_bow;
    fbow::BoWFeatVector fbow_features;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const int level = FeatureLevelFor(levels_up);
        vocabulary_.transform(descriptors, level, fbow_bow, fbow_features);
    }

    double l1_norm = 0.0;
    for (const auto& [word_id, weight] : fbow_bow)
    {
        const auto value = static_cast<WordValue>(weight);
        if (!std::isfinite(value))
        {
            std::cerr << "ORBVocabulary: non-finite word weight" << std::endl;
            return;
        }
        bow->emplace(static_cast<WordId>(word_id), value);
        l1_norm += std::abs(value);
        max_word_id_ = std::max(max_word_id_, static_cast<unsigned int>(word_id));
    }
    if (!std::isfinite(l1_norm) || l1_norm <= 0.0)
    {
        return;
    }
    for (auto& entry : *bow)
    {
        entry.second /= l1_norm;
    }

    for (const auto& [node_id, indices] : fbow_features)
    {
        auto& output = (*features)[static_cast<NodeId>(node_id)];
        output.reserve(indices.size());
        for (const auto index : indices)
        {
            output.push_back(static_cast<unsigned int>(index));
        }
    }
}

void ORBVocabulary::Transform(const std::vector<cv::Mat>& descriptors,
                              BowVector* bow, FeatureVector* features,
                              int levels_up)
{
    if (bow == nullptr || features == nullptr)
    {
        std::cerr << "ORBVocabulary: transform requires non-null outputs"
                  << std::endl;
        return;
    }

    bow->clear();
    features->clear();
    if (descriptors.empty())
    {
        return;
    }

    cv::Mat matrix(static_cast<int>(descriptors.size()), kOrbDescriptorBytes, CV_8U);
    for (std::size_t index = 0; index < descriptors.size(); ++index)
    {
        const cv::Mat& descriptor = descriptors[index];
        if (descriptor.type() != CV_8U ||
            descriptor.total() != kOrbDescriptorBytes)
        {
            std::cerr << "ORBVocabulary: descriptor " << index
                      << " is not 32-byte CV_8U" << std::endl;
            return;
        }
        descriptor.reshape(1, 1).copyTo(matrix.row(static_cast<int>(index)));
    }
    Transform(matrix, bow, features, levels_up);
}
WordValue ORBVocabulary::Score(const BowVector& lhs, const BowVector& rhs) const
{
    auto lhs_it = lhs.begin();
    auto rhs_it = rhs.begin();
    double l1_distance = 0.0;
    while (lhs_it != lhs.end() || rhs_it != rhs.end())
    {
        if (rhs_it == rhs.end() ||
            (lhs_it != lhs.end() && lhs_it->first < rhs_it->first))
        {
            if (!std::isfinite(lhs_it->second))
            {
                return 0.0;
            }
            l1_distance += std::abs(lhs_it->second);
            ++lhs_it;
        }
        else if (lhs_it == lhs.end() || rhs_it->first < lhs_it->first)
        {
            if (!std::isfinite(rhs_it->second))
            {
                return 0.0;
            }
            l1_distance += std::abs(rhs_it->second);
            ++rhs_it;
        }
        else
        {
            if (!std::isfinite(lhs_it->second) || !std::isfinite(rhs_it->second))
            {
                return 0.0;
            }
            l1_distance += std::abs(lhs_it->second - rhs_it->second);
            ++lhs_it;
            ++rhs_it;
        }
    }
    return std::clamp(1.0 - 0.5 * l1_distance, 0.0, 1.0);
}

void ORBVocabulary::clear()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    vocabulary_.clear();
    max_word_id_ = 0;
}

} //namespace ORB_SLAM3

