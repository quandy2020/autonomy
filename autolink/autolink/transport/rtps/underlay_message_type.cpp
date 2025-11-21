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

#include "autolink/transport/rtps/underlay_message_type.hpp"

#include <fastcdr/config.h>
#include <fastcdr/CdrEncoding.hpp>

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"

#include "autolink/common/log.hpp"

namespace autolink {
namespace transport {

UnderlayMessageType::UnderlayMessageType() {
    set_name("UnderlayMessage");
    auto type_size = UnderlayMessage::getMaxCdrSerializedSize();
    type_size += eprosima::fastcdr::Cdr::alignment(
        type_size, 4); /* possible submessage alignment */
    max_serialized_type_size =
        static_cast<uint32_t>(type_size) + 4; /*encapsulation*/
    is_compute_key_provided = UnderlayMessage::isKeyDefined();
    size_t keyLength = UnderlayMessage::getKeyMaxCdrSerializedSize() > 16
                           ? UnderlayMessage::getKeyMaxCdrSerializedSize()
                           : 16;
    m_keyBuffer = (unsigned char*)malloc(keyLength);
    memset(m_keyBuffer, 0, keyLength);
}

UnderlayMessageType::~UnderlayMessageType() {
    if (m_keyBuffer != nullptr) {
        free(m_keyBuffer);
    }
}

bool UnderlayMessageType::serialize(
    const void* const data,
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    eprosima::fastdds::dds::DataRepresentationId_t /*data_representation*/) {
    const UnderlayMessage* p_type =
        reinterpret_cast<const UnderlayMessage*>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(
        reinterpret_cast<char*>(payload.data),
        payload.max_size);  // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer,
                               eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);
    payload.encapsulation =
        ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE
                                                                   : CDR_LE;
    try {
        ser.serialize_encapsulation();
        p_type->serialize(ser);
    } catch (eprosima::fastcdr::exception::Exception& e) {
        AERROR << "serialize exception: " << e.what();
        return false;
    }

    payload.length = static_cast<uint32_t>(ser.get_serialized_data_length());
    return true;
}

bool UnderlayMessageType::deserialize(
    eprosima::fastdds::rtps::SerializedPayload_t& payload, void* data) {
    try {
        // Convert DATA to pointer of your type
        UnderlayMessage* p_type = static_cast<UnderlayMessage*>(data);

        // Object that manages the raw buffer.
        eprosima::fastcdr::FastBuffer fastbuffer(
            reinterpret_cast<char*>(payload.data), payload.length);

        // Object that deserializes the data.
        eprosima::fastcdr::Cdr deser(fastbuffer,
                                     eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);

        // Deserialize encapsulation.
        deser.read_encapsulation();
        payload.encapsulation =
            deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS
                ? CDR_BE
                : CDR_LE;

        // Deserialize the object.
        p_type->deserialize(deser);
    } catch (eprosima::fastcdr::exception::Exception& e) {
        AERROR << "deserialize exception: " << e.what();
        return false;
    }
    return true;
}

uint32_t UnderlayMessageType::calculate_serialized_size(
    const void* const data,
    eprosima::fastdds::dds::DataRepresentationId_t /*data_representation*/) {
    return static_cast<uint32_t>(type::getCdrSerializedSize(
               *static_cast<const UnderlayMessage*>(data))) +
           4u /*encapsulation*/;
}

void* UnderlayMessageType::create_data() {
    return reinterpret_cast<void*>(new UnderlayMessage());
}

void UnderlayMessageType::delete_data(void* data) {
    delete (reinterpret_cast<UnderlayMessage*>(data));
}

bool UnderlayMessageType::compute_key(
    const void* const data, eprosima::fastdds::rtps::InstanceHandle_t& handle,
    bool force_md5) {
    RETURN_VAL_IF(!is_compute_key_provided, false);
    const UnderlayMessage* p_type =
        reinterpret_cast<const UnderlayMessage*>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(
        reinterpret_cast<char*>(m_keyBuffer),
        UnderlayMessage::getKeyMaxCdrSerializedSize());  // Object that manages
                                                         // the raw buffer.
    // Object that serializes the data.
    eprosima::fastcdr::Cdr ser(fastbuffer,
                               eprosima::fastcdr::Cdr::BIG_ENDIANNESS);
    p_type->serializeKey(ser);
    if (force_md5 || UnderlayMessage::getKeyMaxCdrSerializedSize() > 16) {
        m_md5.init();
        m_md5.update(m_keyBuffer, static_cast<unsigned int>(
                                      ser.get_serialized_data_length()));
        m_md5.finalize();
        for (uint8_t i = 0; i < 16; ++i) {
            handle.value[i] = m_md5.digest[i];
        }
    } else {
        for (uint8_t i = 0; i < 16; ++i) {
            handle.value[i] = m_keyBuffer[i];
        }
    }
    return true;
}

bool UnderlayMessageType::compute_key(
    eprosima::fastdds::rtps::SerializedPayload_t& payload,
    eprosima::fastdds::rtps::InstanceHandle_t& ihandle, bool force_md5) {
    UnderlayMessage tmp;
    if (!deserialize(payload, &tmp)) {
        return false;
    }
    return compute_key(&tmp, ihandle, force_md5);
}

}  // namespace transport
}  // namespace autolink