/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License"){}
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


#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace commsgs {
namespace builtin_interfaces {

Time::Time() : sec(0), nanosec(0)
{
}

Time::Time(int32_t seconds, uint32_t nanoseconds)
    : sec{seconds},
      nanosec{nanoseconds}
{
}

bool Time::operator==(const Time& rhs) const
{
    return this->nanosec == rhs.nanosec;
}

bool Time::operator!=(const Time& rhs) const
{
    return !(*this == rhs);
}

bool Time::operator<(const Time& rhs) const
{
    return this->nanosec < rhs.nanosec;
}

bool Time::operator<=(const Time& rhs) const
{
    return this->nanosec <= rhs.nanosec;
}

bool Time::operator>=(const Time& rhs) const
{
    return this->nanosec >= rhs.nanosec;
}

bool Time::operator>(const Time& rhs) const
{
    return this->nanosec > rhs.nanosec;
}

uint32 Time::Nanoseconds() const 
{
    return nanosec;
}

Time Time::Min()
{
    return Time{std::numeric_limits<int32_t>::min(), 0}; 
}

Time Time::Max()
{
    return Time(std::numeric_limits<int32_t>::max(), 999999999);
}

double Time::Seconds() const
{
    return sec;
}

Time Time::Now()
{
    // 直接操作时间戳（避免多次类型转换）
    const auto now = std::chrono::system_clock::now();
    const auto duration = now.time_since_epoch();
    const int64_t total_ns = duration.count();
     
     return Time{
         .sec      = static_cast<int32_t>(total_ns / 1'000'000'000),
         .nanosec  = static_cast<uint32_t>(total_ns % 1'000'000'000)
     };
}

proto::builtin_interfaces::Time ToProto(const Time& data)
{
    proto::builtin_interfaces::Time proto;
    return proto;
}

Time FromProto(const proto::builtin_interfaces::Time& proto)
{
    Time data;
    return data;
}


proto::builtin_interfaces::Duration ToProto(const Duration& Duration)
{
    proto::builtin_interfaces::Duration proto;
    return proto;
}


Duration FromProto(const proto::builtin_interfaces::Duration& proto)
{
    Duration data;
    return data;
}

}  // namespace builtin_interfaces
}  // namespace commsgs
}  // namespace autonomy