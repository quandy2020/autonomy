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

#include "autolink/base/macros.hpp"

#include <fastcdr/config.h>
#include <fastdds/utils/md5.hpp>
#include "fastdds/dds/topic/TopicDataType.hpp"

#include "autolink/transport/rtps/underlay_message.hpp"

namespace autolink {
namespace transport {

/*!
 * @brief This class represents the TopicDataType of the type UnderlayMessage
 * defined by the user in the IDL file.
 * @ingroup UNDERLAYMESSAGE
 */
class UnderlayMessageType : public eprosima::fastdds::dds::TopicDataType
{
public:
    using type = UnderlayMessage;

    UnderlayMessageType();
    virtual ~UnderlayMessageType();
    bool serialize(const void* const data,
                   eprosima::fastdds::rtps::SerializedPayload_t& payload,
                   eprosima::fastdds::dds::DataRepresentationId_t
                       data_representation) override;

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

    void* create_data() override;           // NOLINT
    void delete_data(void* data) override;  // NOLINT
    eprosima::fastdds::MD5 m_md5;
    unsigned char* m_keyBuffer;
};

}  // namespace transport
}  // namespace autolink