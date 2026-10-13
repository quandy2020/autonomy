/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "autolink/proto/qos_profile.pb.h"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastdds/dds/subscriber/qos/DataReaderQos.hpp"

namespace autolink {
namespace transport {

using proto::QosDurabilityPolicy;
using proto::QosHistoryPolicy;
using proto::QosProfile;
using proto::QosReliabilityPolicy;

/**
 * Maps autolink QosProfile onto Fast DDS 3.x DataWriter/DataReader QoS.
 * Covers History (kind + depth), Reliability, Durability.
 */
class AttributesFiller
{
public:
    AttributesFiller() = default;
    ~AttributesFiller() = default;

    static bool FillInPubQos(const QosProfile& qos,
                             eprosima::fastdds::dds::DataWriterQos* wqos);

    static bool FillInSubQos(const QosProfile& qos,
                             eprosima::fastdds::dds::DataReaderQos* rqos);
};

}  // namespace transport
}  // namespace autolink
