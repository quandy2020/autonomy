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


#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace commsgs {
namespace builtin_interfaces {


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