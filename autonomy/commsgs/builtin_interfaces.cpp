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
        static_cast<int32_t>(total_ns / 1'000'000'000),
        static_cast<uint32_t>(total_ns % 1'000'000'000)
     };
}

proto::builtin_interfaces::Time ToProto(const Time& data)
{
    proto::builtin_interfaces::Time proto;
    proto.set_sec(data.sec);
    proto.set_nanosec(data.nanosec);
    return proto;
}

Time FromProto(const proto::builtin_interfaces::Time& proto)
{
    return {
        proto.sec(),
        proto.nanosec()
    };
}

Duration::Duration(int32_t seconds, uint32_t nanoseconds)
{
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(seconds)).count();
    duration_  += nanoseconds;
}

Duration::Duration(std::chrono::nanoseconds nanoseconds)
{
    duration_ = nanoseconds.count();
}

Duration& Duration::operator=(const Duration& rhs) = default;

bool Duration::operator==(const Duration& rhs) const
{
    return duration_ == duration_;
}

bool Duration::operator!=(const Duration& rhs) const
{
    return duration_ != rhs.duration_;
}

bool Duration::operator<(const Duration & rhs) const
{
    return duration_ < rhs.duration_;
}

bool Duration::operator<=(const Duration& rhs) const
{
    return duration_ <= rhs.duration_;
}

bool Duration::operator>=(const Duration& rhs) const
{
    return duration_ >= rhs.duration_;
}

bool Duration::operator>(const Duration& rhs) const
{
    return duration_ > rhs.duration_;
}

void bounds_check_duration_sum(int64_t lhsns, int64_t rhsns, uint64_t max)
{
  auto abs_lhs = static_cast<uint64_t>(std::abs(lhsns));
  auto abs_rhs = static_cast<uint64_t>(std::abs(rhsns));

  if (lhsns > 0 && rhsns > 0) {
    if (abs_lhs + abs_rhs > max) {
      throw std::overflow_error("addition leads to int64_t overflow");
    }
  } else if (lhsns < 0 && rhsns < 0) {
    if (abs_lhs + abs_rhs > max) {
      throw std::underflow_error("addition leads to int64_t underflow");
    }
  }
}

Duration Duration::operator+(const Duration& rhs) const
{
    bounds_check_duration_sum(
        this->duration_,
        rhs.duration_,
        std::numeric_limits<int64_t>::max());
    return Duration::FromNanoseconds(duration_ + rhs.duration_);
}

void
bounds_check_duration_difference(int64_t lhsns, int64_t rhsns, uint64_t max)
{
  auto abs_lhs = static_cast<uint64_t>(std::abs(lhsns));
  auto abs_rhs = static_cast<uint64_t>(std::abs(rhsns));

  if (lhsns > 0 && rhsns < 0) {
    if (abs_lhs + abs_rhs > max) {
      throw std::overflow_error("duration subtraction leads to int64_t overflow");
    }
  } else if (lhsns < 0 && rhsns > 0) {
    if (abs_lhs + abs_rhs > max) {
      throw std::underflow_error("duration subtraction leads to int64_t underflow");
    }
  }
}

Duration Duration::operator-(const Duration& rhs) const
{
  bounds_check_duration_difference(
    this->duration_, rhs.duration_,
    std::numeric_limits<int64_t>::max());

  return Duration::FromNanoseconds(duration_ - rhs.duration_);
}

void bounds_check_duration_scale(int64_t dns, double scale, uint64_t max)
{
  auto abs_dns = static_cast<uint64_t>(std::abs(dns));
  auto abs_scale = std::abs(scale);
  if (abs_scale > 1.0 && abs_dns >
    static_cast<uint64_t>(static_cast<long double>(max) / static_cast<long double>(abs_scale)))
  {
    if ((dns > 0 && scale > 0) || (dns < 0 && scale < 0)) {
      throw std::overflow_error("duration scaling leads to int64_t overflow");
    } else {
      throw std::underflow_error("duration scaling leads to int64_t underflow");
    }
  }
}

Duration
Duration::operator*(double scale) const
{
  if (!std::isfinite(scale)) {
    throw std::runtime_error("abnormal scale in rclcpp::Duration");
  }
  bounds_check_duration_scale(
    this->duration_, scale,
    std::numeric_limits<int64_t>::max());
  long double scale_ld = static_cast<long double>(scale);
  return Duration::FromNanoseconds(
    static_cast<int64_t>(
      static_cast<long double>(duration_) * scale_ld));
}

int64_t Duration::Nanoseconds() const
{
    return duration_;
}

Duration Duration::Max()
{
    return Duration(std::numeric_limits<int32_t>::max(), 999999999);
}

double Duration::Seconds() const
{
    return std::chrono::duration<double>(std::chrono::nanoseconds(duration_)).count();
}

Duration Duration::FromSeconds(double seconds)
{
  Duration ret;
  ret.duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds)).count();
  return ret;
}

Duration Duration::FromNanoseconds(int64_t nanoseconds)
{
    Duration ret;
    ret.duration_ = nanoseconds;
    return ret;
}

proto::builtin_interfaces::Duration ToProto(const Duration& data)
{
    proto::builtin_interfaces::Duration proto;
    // *proto.mutable_stamp() = ToProto(data.stamp);
    // proto.set_frame_id(data.frame_id);
    return proto;
}

Duration FromProto(const proto::builtin_interfaces::Duration& proto)
{
    Duration data;
    // data.stamp = FromProto(proto.stamp());
    // data.frame_id = proto.frame_id();
    return data;
}

}  // namespace builtin_interfaces
}  // namespace commsgs
}  // namespace autonomy