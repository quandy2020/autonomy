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

#include <cstdint>
#include <string>
#include <utility>

#include "fastcdr/Cdr.h"

namespace autolink {
namespace transport {

/**
 * FastCDR-serializable payload wrapper for RTPS topics.
 * Business bytes live in data(), optionally prefixed by MessageInfo
 * (see message_info_prefix.hpp).
 */
class UnderlayMessage
{
public:
    UnderlayMessage();
    ~UnderlayMessage();

    UnderlayMessage(const UnderlayMessage& x);
    UnderlayMessage(UnderlayMessage&& x);

    UnderlayMessage& operator=(const UnderlayMessage& x);
    UnderlayMessage& operator=(UnderlayMessage&& x);

    inline void timestamp(int32_t timestamp) {
        m_timestamp = timestamp;
    }
    inline int32_t timestamp() const {
        return m_timestamp;
    }
    inline int32_t& timestamp() {
        return m_timestamp;
    }

    inline void seq(int32_t seq) {
        m_seq = seq;
    }
    inline int32_t seq() const {
        return m_seq;
    }
    inline int32_t& seq() {
        return m_seq;
    }

    inline void data(const std::string& data) {
        m_data = data;
    }
    inline void data(std::string&& data) {
        m_data = std::move(data);
    }
    inline const std::string& data() const {
        return m_data;
    }
    inline std::string& data() {
        return m_data;
    }

    inline void datatype(const std::string& datatype) {
        m_datatype = datatype;
    }
    inline void datatype(std::string&& datatype) {
        m_datatype = std::move(datatype);
    }
    inline const std::string& datatype() const {
        return m_datatype;
    }
    inline std::string& datatype() {
        return m_datatype;
    }

    static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);
    static size_t getCdrSerializedSize(const UnderlayMessage& data,
                                       size_t current_alignment = 0);

    void serialize(eprosima::fastcdr::Cdr& cdr) const;
    void deserialize(eprosima::fastcdr::Cdr& cdr);

    static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);
    static bool isKeyDefined();
    void serializeKey(eprosima::fastcdr::Cdr& cdr) const;

private:
    int32_t m_timestamp;
    int32_t m_seq;
    std::string m_data;
    std::string m_datatype;
};

}  // namespace transport
}  // namespace autolink
