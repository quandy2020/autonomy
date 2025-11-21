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

// Fix FastCDR TEMPLATE_SPEC issue - must be included before any FastCDR headers
#include <fastcdr/config.h>

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/base/macros.hpp"

#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/SampleInfo.hpp"
#include "fastdds/dds/subscriber/SubscriberListener.hpp"
#include "fastdds/dds/topic/TopicDescription.hpp"

#include "autolink/common/util.hpp"
#include "autolink/transport/common/common_type.hpp"
#include "autolink/transport/message/message_info.hpp"
#include "autolink/transport/rtps/underlay_message.hpp"
#include "autolink/transport/rtps/underlay_message_type.hpp"

namespace autolink {
namespace transport {
namespace dispatcher {

class SubscriberListener : public eprosima::fastdds::dds::SubscriberListener
{
public:
    explicit SubscriberListener(const rtps::subsciber_callback& callback);
    virtual ~SubscriberListener();

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;
    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader* reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
        override;  // NOLINT

private:
    rtps::subsciber_callback callback_;
    MessageInfo msg_info_;
    std::mutex mutex_;
};

}  // namespace dispatcher
}  // namespace transport
}  // namespace autolink