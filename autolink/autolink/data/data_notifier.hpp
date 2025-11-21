/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "autolink/common/log.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/data/cache_buffer.hpp"
#include "autolink/event/perf_event_cache.hpp"
#include "autolink/time/time.hpp"

namespace autolink {
namespace data {

using autolink::Time;
using autolink::base::AtomicHashMap;
using autolink::event::PerfEventCache;

struct Notifier {
    std::function<void()> callback;
};

class DataNotifier
{
public:
    using NotifyVector = std::vector<std::shared_ptr<Notifier>>;
    ~DataNotifier() {}

    void AddNotifier(uint64_t channel_id,
                     const std::shared_ptr<Notifier>& notifier);

    bool Notify(const uint64_t channel_id);

private:
    std::mutex notifies_map_mutex_;
    AtomicHashMap<uint64_t, NotifyVector> notifies_map_;

    DECLARE_SINGLETON(DataNotifier)
};

inline DataNotifier::DataNotifier() {}

inline void DataNotifier::AddNotifier(
    uint64_t channel_id, const std::shared_ptr<Notifier>& notifier) {
    std::lock_guard<std::mutex> lock(notifies_map_mutex_);
    NotifyVector* notifies = nullptr;
    if (notifies_map_.Get(channel_id, &notifies)) {
        notifies->emplace_back(notifier);
    } else {
        NotifyVector new_notify = {notifier};
        notifies_map_.Set(channel_id, new_notify);
    }
}

inline bool DataNotifier::Notify(const uint64_t channel_id) {
    NotifyVector* notifies = nullptr;
    if (notifies_map_.Get(channel_id, &notifies)) {
        for (auto& notifier : *notifies) {
            if (notifier && notifier->callback) {
                notifier->callback();
            }
        }
        return true;
    }
    return false;
}

}  // namespace data
}  // namespace autolink