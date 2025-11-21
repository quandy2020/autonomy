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

#include "autolink/transport/shm/notifier_factory.hpp"

#include <string>

#include "autolink/common/global_data.hpp"
#include "autolink/common/log.hpp"
#include "autolink/transport/shm/condition_notifier.hpp"
#include "autolink/transport/shm/multicast_notifier.hpp"

namespace autolink {
namespace transport {

using common::GlobalData;

auto NotifierFactory::CreateNotifier() -> NotifierPtr {
    std::string notifier_type(ConditionNotifier::Type());
    auto& g_conf = GlobalData::Instance()->Config();
    if (g_conf.has_transport_conf() && g_conf.transport_conf().has_shm_conf() &&
        !g_conf.transport_conf().shm_conf().notifier_type().empty()) {
        notifier_type = g_conf.transport_conf().shm_conf().notifier_type();
    }

    ADEBUG << "notifier type: " << notifier_type;

    if (notifier_type == MulticastNotifier::Type()) {
        return CreateMulticastNotifier();
    } else if (notifier_type == ConditionNotifier::Type()) {
        return CreateConditionNotifier();
    }

    AINFO << "unknown notifier, we use default notifier: " << notifier_type;
    return CreateConditionNotifier();
}

auto NotifierFactory::CreateConditionNotifier() -> NotifierPtr {
    return ConditionNotifier::Instance();
}

auto NotifierFactory::CreateMulticastNotifier() -> NotifierPtr {
    return MulticastNotifier::Instance();
}

}  // namespace transport
}  // namespace autolink
