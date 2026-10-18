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

#include "argus/relocalization/relocalization_manager.hpp"

#include <utility>

namespace argus {
namespace relocalization {

RelocalizationManager::RelocalizationManager(RelocalizationConfig config)
    : config_(std::move(config)), pool_(config_.thread_pool) {
    engine_ = BbsSearchEngine::make_shared(nullptr, config_.branch_and_bound);
}

RelocalizationManager::~RelocalizationManager() {
    Stop();
}

void RelocalizationManager::Start() {
    if (!config_.enable) {
        return;
    }
    pool_.Start();
}

void RelocalizationManager::Stop() {
    pool_.Stop();
}

void RelocalizationManager::SetPriorMap(PriorOccupancyMap::ConstSharedPtr map) {
    std::lock_guard<std::mutex> lock(engine_mutex_);
    if (engine_) {
        engine_->SetMap(std::move(map));
    }
}

bool RelocalizationManager::HasPriorMap() const {
    std::lock_guard<std::mutex> lock(engine_mutex_);
    return engine_ && engine_->HasMap();
}

bool RelocalizationManager::RequestRelocalization(RelocRequest request) {
    if (!config_.enable || !pool_.running()) {
        return false;
    }
    if (request.query_cloud.empty()) {
        return false;
    }
    BbsSearchEngine::SharedPtr engine;
    {
        std::lock_guard<std::mutex> lock(engine_mutex_);
        if (!engine_ || !engine_->HasMap()) {
            return false;
        }
        engine = engine_;
    }
    if (request.id == 0) {
        request.id = next_id_.fetch_add(1);
    }
    const uint64_t rid = request.id;
    // Copy request into task (off ESIKF after Submit returns).
    return pool_.Submit([this, engine, req = std::move(request),
                         rid](std::atomic<bool>& cancel) mutable {
        RelocResult result;
        if (!engine) {
            result.request_id = rid;
            result.cancelled = true;
        } else {
            // Do not hold engine_mutex_ across Search (SetPriorMap may need
            // it).
            result = engine->Search(req, &cancel);
        }
        if (result.success) {
            success_count_.fetch_add(1);
        }
        std::lock_guard<std::mutex> lock(results_mutex_);
        results_.push_back(std::move(result));
    });
}

bool RelocalizationManager::TryAutoTrigger(bool lost, double stamp_sec,
                                           RelocRequest request) {
    if (!config_.enable || config_.trigger != "tracking_lost") {
        return false;
    }
    if (!lost) {
        lost_streak_ = 0;
        return false;
    }
    ++lost_streak_;
    if (lost_streak_ < config_.min_lost_streak) {
        return false;
    }
    if (stamp_sec - last_auto_request_sec_ < config_.cooldown_sec) {
        return false;
    }
    if (!RequestRelocalization(std::move(request))) {
        return false;
    }
    last_auto_request_sec_ = stamp_sec;
    ++auto_trigger_count_;
    // Keep streak so cooldown gate remains active until recovery.
    return true;
}

std::vector<RelocResult> RelocalizationManager::TakeResults() {
    std::lock_guard<std::mutex> lock(results_mutex_);
    std::vector<RelocResult> out;
    out.swap(results_);
    return out;
}

void RelocalizationManager::CancelPending() {
    pool_.CancelAll();
}

int RelocalizationManager::pending_count() const {
    return static_cast<int>(pool_.queue_size());
}

int RelocalizationManager::dropped_count() const {
    return pool_.dropped_count();
}

}  // namespace relocalization
}  // namespace argus
