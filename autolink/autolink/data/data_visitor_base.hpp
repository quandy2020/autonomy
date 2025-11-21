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

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "autolink/common/global_data.hpp"
#include "autolink/common/log.hpp"
#include "autolink/data/data_notifier.hpp"

namespace autolink {
namespace data {

class DataVisitorBase
{
public:
    DataVisitorBase() : notifier_(new Notifier()) {}

    void RegisterNotifyCallback(std::function<void()>&& callback) {
        notifier_->callback = callback;
    }

protected:
    DataVisitorBase(const DataVisitorBase&) = delete;
    DataVisitorBase& operator=(const DataVisitorBase&) = delete;

    uint64_t next_msg_index_ = 0;
    DataNotifier* data_notifier_ = DataNotifier::Instance();
    std::shared_ptr<Notifier> notifier_;
};

}  // namespace data
}  // namespace autolink
