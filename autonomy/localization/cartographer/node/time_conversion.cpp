/*
 * Copyright 2016 The Cartographer Authors
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

#include "autonomy/localization/cartographer/node/time_conversion.hpp"

#include "autonomy/localization/cartographer/common/time.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

commsgs::builtin_interfaces::Time ToCommsgs(
    const ::cartographer::common::Time time) {
    const int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
    const int64_t ns_since_unix_epoch =
        (uts_timestamp -
         ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
             10000000ll) *
        100ll;
    commsgs::builtin_interfaces::Time result;
    result.sec = static_cast<int32_t>(ns_since_unix_epoch / 1000000000ll);
    result.nanosec =
        static_cast<uint32_t>(ns_since_unix_epoch % 1000000000ll);
    return result;
}

::cartographer::common::Time FromCommsgs(
    const commsgs::builtin_interfaces::Time& time) {
    const int64_t ns_since_unix_epoch =
        static_cast<int64_t>(time.sec) * 1000000000ll +
        static_cast<int64_t>(time.nanosec);
    return ::cartographer::common::FromUniversal(
        ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
            10000000ll +
        (ns_since_unix_epoch + 50) / 100);
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
