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

#include "autolink/transport/rtps/underlay_message.hpp"
#include "fastdds/dds/topic/TopicDataType.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"
#include "fastdds/rtps/common/InstanceHandle.hpp"
#include "fastdds/rtps/common/SerializedPayload.hpp"
#include "fastdds/utils/md5.hpp"

namespace autolink {
namespace transport {

/**
 * TopicDataType / TypeSupport for UnderlayMessage (Fast DDS 3.x).
 */
class UnderlayMessageType : public eprosima::fastdds::dds::TopicDataType
{
public:
    using type = UnderlayMessage;

    UnderlayMessageType();
    ~UnderlayMessageType() override;

    bool serialize(
            const void* const data,
            eprosima::fastdds::rtps::SerializedPayload_t& payload,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation)
            override;
    bool deserialize(eprosima::fastdds::rtps::SerializedPayload_t& payload,
                     void* data) override;
    uint32_t calculate_serialized_size(
            const void* const data,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation)
            override;
    bool compute_key(eprosima::fastdds::rtps::SerializedPayload_t& payload,
                     eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
                     bool force_md5 = false) override;
    bool compute_key(const void* const data,
                     eprosima::fastdds::rtps::InstanceHandle_t& ihandle,
                     bool force_md5 = false) override;
    void* create_data() override;
    void delete_data(void* data) override;

    eprosima::fastdds::MD5 m_md5;
    unsigned char* m_keyBuffer;
};

/** Factory used by Participant (declared there without this header). */
eprosima::fastdds::dds::TypeSupport MakeUnderlayTypeSupport();

}  // namespace transport
}  // namespace autolink
