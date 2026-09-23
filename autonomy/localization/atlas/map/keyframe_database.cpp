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
 * @file keyframe_database.cpp
 * @brief `KeyFrameDatabase` implementation: inverted index and BoW candidate retrieval.
 *
 * Corresponds to ORB-SLAM3 `KeyFrameDatabase.cc`; scoring uses FBoW.
 */

#include "autonomy/localization/atlas/map/keyframe_database.hpp"

#include <algorithm>
#include <set>
#include <unordered_map>

namespace autonomy {
namespace localization {
namespace atlas {

KeyFrameDatabase::KeyFrameDatabase(
    std::shared_ptr<feature::OrbVocabulary> vocabulary)
    : vocabulary_(std::move(vocabulary)) {}

void KeyFrameDatabase::Add(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe || !keyframe->HasBoW()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [word_id, weight] : keyframe->bow_vector()) {
        (void)weight;
        inverted_file_[word_id].push_back(keyframe);
    }
}

void KeyFrameDatabase::Erase(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe || !keyframe->HasBoW()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [word_id, weight] : keyframe->bow_vector()) {
        (void)weight;
        auto it = inverted_file_.find(word_id);
        if (it == inverted_file_.end()) {
            continue;
        }
        it->second.remove_if([&](const std::weak_ptr<KeyFrame>& weak) {
            auto locked = weak.lock();
            return !locked || locked == keyframe;
        });
    }
}

void KeyFrameDatabase::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    inverted_file_.clear();
}

std::vector<std::shared_ptr<KeyFrame>>
KeyFrameDatabase::DetectRelocalizationCandidates(
    const tracking::Frame& frame) const {
    if (!frame.HasBoW() || !vocabulary_ || !vocabulary_->is_valid()) {
        return {};
    }

    std::list<std::shared_ptr<KeyFrame>> sharing;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::unordered_map<KeyFrame*, int> word_counts;
        for (const auto& [word_id, weight] : frame.bow_vector()) {
            (void)weight;
            const auto it = inverted_file_.find(word_id);
            if (it == inverted_file_.end()) {
                continue;
            }
            for (const auto& weak : it->second) {
                auto keyframe = weak.lock();
                if (!keyframe || keyframe->isBad()) {
                    continue;
                }
                ++word_counts[keyframe.get()];
                if (word_counts[keyframe.get()] == 1) {
                    sharing.push_back(keyframe);
                }
            }
        }
        // Attach counts onto keyframes via temporary map return path.
        int max_common = 0;
        std::vector<std::pair<std::shared_ptr<KeyFrame>, int>> counted;
        counted.reserve(sharing.size());
        for (const auto& keyframe : sharing) {
            const int count = word_counts[keyframe.get()];
            counted.emplace_back(keyframe, count);
            max_common = std::max(max_common, count);
        }
        const int min_common = static_cast<int>(0.8f * max_common);
        std::vector<std::pair<float, std::shared_ptr<KeyFrame>>> scored;
        for (const auto& [keyframe, count] : counted) {
            if (count <= min_common) {
                continue;
            }
            const float score = static_cast<float>(
                vocabulary_->Score(frame.bow_vector(), keyframe->bow_vector()));
            scored.emplace_back(score, keyframe);
        }
        std::sort(scored.begin(), scored.end(),
                  [](const auto& a, const auto& b) { return a.first > b.first; });
        std::vector<std::shared_ptr<KeyFrame>> result;
        result.reserve(std::min<size_t>(scored.size(), 10));
        for (size_t i = 0; i < scored.size() && i < 10; ++i) {
            result.push_back(scored[i].second);
        }
        return result;
    }
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrameDatabase::DetectLoopCandidates(
    const std::shared_ptr<KeyFrame>& keyframe, float min_score) const {
    if (!keyframe || !keyframe->HasBoW() || !vocabulary_ ||
        !vocabulary_->is_valid()) {
        return {};
    }

    std::set<std::shared_ptr<KeyFrame>> connected;
    for (const auto& [weak, weight] : keyframe->GetConnectedKeyFrames()) {
        (void)weight;
        if (auto other = weak.lock()) {
            connected.insert(other);
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<KeyFrame*, int> word_counts;
    std::list<std::shared_ptr<KeyFrame>> sharing;
    for (const auto& [word_id, weight] : keyframe->bow_vector()) {
        (void)weight;
        const auto it = inverted_file_.find(word_id);
        if (it == inverted_file_.end()) {
            continue;
        }
        for (const auto& weak : it->second) {
            auto other = weak.lock();
            if (!other || other->isBad() || other == keyframe ||
                connected.count(other)) {
                continue;
            }
            ++word_counts[other.get()];
            if (word_counts[other.get()] == 1) {
                sharing.push_back(other);
            }
        }
    }

    int max_common = 0;
    for (const auto& other : sharing) {
        max_common = std::max(max_common, word_counts[other.get()]);
    }
    const int min_common = static_cast<int>(0.8f * max_common);

    std::vector<std::pair<float, std::shared_ptr<KeyFrame>>> scored;
    for (const auto& other : sharing) {
        if (word_counts[other.get()] <= min_common) {
            continue;
        }
        const float score = static_cast<float>(
            vocabulary_->Score(keyframe->bow_vector(), other->bow_vector()));
        if (score >= min_score) {
            scored.emplace_back(score, other);
        }
    }
    std::sort(scored.begin(), scored.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<std::shared_ptr<KeyFrame>> result;
    for (size_t i = 0; i < scored.size() && i < 10; ++i) {
        result.push_back(scored[i].second);
    }
    return result;
}

void KeyFrameDatabase::DetectNBestCandidates(
    const std::shared_ptr<KeyFrame>& keyframe,
    std::vector<std::shared_ptr<KeyFrame>>* loop_cands,
    std::vector<std::shared_ptr<KeyFrame>>* merge_cands,
    int num_candidates) const {
    if (loop_cands == nullptr || merge_cands == nullptr || !keyframe ||
        !keyframe->HasBoW() || !vocabulary_ || !vocabulary_->is_valid() ||
        num_candidates <= 0) {
        return;
    }
    loop_cands->clear();
    merge_cands->clear();

    std::set<std::shared_ptr<KeyFrame>> connected;
    for (const auto& [weak, weight] : keyframe->GetConnectedKeyFrames()) {
        (void)weight;
        if (auto other = weak.lock()) {
            connected.insert(other);
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<KeyFrame*, int> word_counts;
    std::unordered_map<KeyFrame*, float> scores;
    std::list<std::shared_ptr<KeyFrame>> sharing;
    for (const auto& [word_id, weight] : keyframe->bow_vector()) {
        (void)weight;
        const auto it = inverted_file_.find(word_id);
        if (it == inverted_file_.end()) {
            continue;
        }
        for (const auto& weak : it->second) {
            auto other = weak.lock();
            if (!other || other->isBad() || other == keyframe ||
                connected.count(other)) {
                continue;
            }
            ++word_counts[other.get()];
            if (word_counts[other.get()] == 1) {
                sharing.push_back(other);
            }
        }
    }
    if (sharing.empty()) {
        return;
    }

    int max_common = 0;
    for (const auto& other : sharing) {
        max_common = std::max(max_common, word_counts[other.get()]);
    }
    const int min_common = static_cast<int>(0.8f * max_common);

    std::vector<std::pair<float, std::shared_ptr<KeyFrame>>> scored;
    for (const auto& other : sharing) {
        if (word_counts[other.get()] <= min_common) {
            continue;
        }
        const float score = static_cast<float>(
            vocabulary_->Score(keyframe->bow_vector(), other->bow_vector()));
        scores[other.get()] = score;
        scored.emplace_back(score, other);
    }
    if (scored.empty()) {
        return;
    }

    // Accumulate covisibility scores → group by representative KF (full bucketing).
    std::unordered_map<KeyFrame*, float> group_acc;
    std::unordered_map<KeyFrame*, std::shared_ptr<KeyFrame>> group_rep;
    float best_acc = 0.f;
    for (const auto& [score, other] : scored) {
        float best_score = score;
        float acc = score;
        auto best_kf = other;
        for (const auto& neigh : other->GetBestCovisibilityKeyFrames(10)) {
            if (!neigh) {
                continue;
            }
            const auto sit = scores.find(neigh.get());
            if (sit == scores.end()) {
                continue;
            }
            acc += sit->second;
            if (sit->second > best_score) {
                best_score = sit->second;
                best_kf = neigh;
            }
        }
        KeyFrame* key = best_kf.get();
        auto git = group_acc.find(key);
        if (git == group_acc.end() || acc > git->second) {
            group_acc[key] = acc;
            group_rep[key] = best_kf;
        }
        best_acc = std::max(best_acc, acc);
    }

    std::vector<std::pair<float, std::shared_ptr<KeyFrame>>> acc_scored;
    acc_scored.reserve(group_acc.size());
    const float min_acc = 0.75f * best_acc;
    for (const auto& [key, acc] : group_acc) {
        if (acc < min_acc) {
            continue;
        }
        acc_scored.emplace_back(acc, group_rep[key]);
    }
    std::sort(acc_scored.begin(), acc_scored.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    std::set<KeyFrame*> added;
    Map* cur_map = keyframe->GetMap();
    for (const auto& [acc, cand] : acc_scored) {
        (void)acc;
        if (!cand || cand->isBad() || added.count(cand.get())) {
            continue;
        }
        Map* cand_map = cand->GetMap();
        if (cand_map == cur_map) {
            if (static_cast<int>(loop_cands->size()) < num_candidates) {
                loop_cands->push_back(cand);
                added.insert(cand.get());
            }
        } else if (cand_map != nullptr) {
            if (static_cast<int>(merge_cands->size()) < num_candidates) {
                merge_cands->push_back(cand);
                added.insert(cand.get());
            }
        }
        if (static_cast<int>(loop_cands->size()) >= num_candidates &&
            static_cast<int>(merge_cands->size()) >= num_candidates) {
            break;
        }
    }
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
