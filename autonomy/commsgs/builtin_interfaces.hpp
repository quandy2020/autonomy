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
#include <chrono>

#include "autonomy/common/port.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/proto/builtin_interfaces.pb.h"

namespace autonomy {
namespace commsgs {
namespace builtin_interfaces {

struct Time 
{
    // Define SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(Time)

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
    // Define SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(Duration)

    Duration() = default;

    /**
     * Initializes the time values for seconds and nanoseconds individually.
     * Large values for nsecs are wrapped automatically with the remainder added to secs.
     * Both inputs must be integers.
     * Seconds can be negative.
     *
     * \param seconds time in seconds
     * \param nanoseconds time in nanoseconds
     */
    Duration(int32_t seconds, uint32_t nanoseconds);

    /// Construct duration from the specified std::chrono::nanoseconds.
    explicit Duration(std::chrono::nanoseconds nanoseconds);


    // This constructor matches any std::chrono value other than nanoseconds
    // intentionally not using explicit to create a conversion constructor
    template<class Rep, class Period>
    // cppcheck-suppress noExplicitConstructor
    Duration(const std::chrono::duration<Rep, Period> & duration)  // NOLINT(runtime/explicit)
        : Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration))
    {}

    // Duration(const Duration& rhs);

    // cppcheck-suppress operatorEq // this is a false positive from cppcheck
    Duration& operator=(const Duration& rhs);

    bool operator==(const Duration& rhs) const;

    bool operator!=(const Duration& rhs) const;

    bool operator<(const Duration& rhs) const;

    bool operator<=(const Duration& rhs) const;

    bool operator>=(const Duration& rhs) const;

    bool operator>(const Duration& rhs) const;

    Duration operator+(const Duration& rhs) const;

    Duration operator-(const Duration& rhs) const;

    Duration operator*(double scale) const;

    /// Get the maximum representable value.
    /**
     * \return the maximum representable value
     */
    static Duration Max();

    /// Get duration in nanosecods
    /**
     * \return the duration in nanoseconds.
     */
    int64_t Nanoseconds() const;
    
    /// Get duration in seconds
    /**
     * \warning Depending on sizeof(double) there could be significant precision loss.
     *   When an exact time is required use nanoseconds() instead.
     * \return the duration in seconds as a floating point number.
     */
    double Seconds() const;

    /// Create a duration object from a floating point number representing seconds
    static Duration FromSeconds(double seconds);

    /// Create a duration object from an integer number representing nanoseconds
    static Duration FromNanoseconds(int64_t nanoseconds);

    /// Convert Duration into a std::chrono::Duration.
    template<class DurationT>
    DurationT ToChrono() const
    {
        return std::chrono::duration_cast<DurationT>(std::chrono::nanoseconds(this->Nanoseconds()));
    }

    // Nanoseconds
    int64_t duration_;
};

/******************************************** PROTO <--> Commsgs ************************************************/

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

using  Time = commsgs::builtin_interfaces::Time;

}  // namespace autonomy
