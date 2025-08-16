/*
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

#include <vector>
#include <string>

#include "autonomy/common/port.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/proto/builtin_interfaces.pb.h"

namespace autonomy {
namespace commsgs {
namespace builtin_interfaces {

struct Time 
{
    Time();

    Time(int32_t seconds, uint32_t nanoseconds);

    Time(const Time& rhs) = default;

    Time& operator=(const Time& rhs)  = default;

    bool operator==(const Time& rhs) const;

    bool operator!=(const Time& rhs) const;

    bool operator<(const Time& rhs) const;

    bool operator<=(const Time& rhs) const;

    bool operator>=(const Time& rhs) const;

    bool operator>(const Time& rhs) const;

    uint32 Nanoseconds() const;

    static Time Min();

    static Time Max();

    static Time Now();

    double Seconds() const;

    // The seconds component, valid over all int32 values.
    int32 sec;

    // The nanoseconds component, valid in the range [0, 10e9).
    uint32 nanosec;
};
    
// Standard metadata for higher-level stamped data types.
// This is generally used to communicate timestamped data
// in a particular coordinate frame.
struct Duration 
{
    // Two-integer timestamp that is expressed as seconds and nanoseconds.
    Time stamp;

    // Transform frame with which this data is associated.
    std::string frame_id;
};

// Converts 'data' to a proto::builtin_interfaces::Time.
proto::builtin_interfaces::Time ToProto(const Time& data);

// Converts 'proto' to Time.
Time FromProto(const proto::builtin_interfaces::Time& proto);

// Converts 'data' to a proto::builtin_interfaces::Duration.
proto::builtin_interfaces::Duration ToProto(const Duration& data);

// Converts 'proto' to Time.
Duration FromProto(const proto::builtin_interfaces::Duration& proto);

}  // namespace builtin_interfaces
}  // namespace commsgs

}  // namespace autonomy
