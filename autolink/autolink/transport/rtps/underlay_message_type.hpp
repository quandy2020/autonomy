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

#include "autolink/transport/rtps/underlay_message.hpp"
#include "fastdds/dds/topic/TopicDataType.hpp"
#include "fastrtps/fastrtps/utils/md5.h"

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
    virtual bool serialize(  // NOLINT
        void* data,
        eprosima::fastrtps::rtps::SerializedPayload_t* payload) override;
    virtual bool deserialize(  // NOLINT
        eprosima::fastrtps::rtps::SerializedPayload_t* payload,
        void* data) override;
    virtual std::function<uint32_t()> getSerializedSizeProvider(  // NOLINT
        void* data) override;
    virtual bool getKey(void* data,  // NOLINT
                        eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
                        bool force_md5 = false) override;
    virtual void* createData() override;           // NOLINT
    virtual void deleteData(void* data) override;  // NOLINT
    MD5 m_md5;
    unsigned char* m_keyBuffer;
};

}  // namespace transport
}  // namespace autolink
