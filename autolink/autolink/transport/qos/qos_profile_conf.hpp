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

#include <cstdint>

#include "autolink/proto/qos_profile.pb.h"

namespace autolink {
namespace transport {

using autolink::proto::QosDurabilityPolicy;
using autolink::proto::QosHistoryPolicy;
using autolink::proto::QosProfile;
using autolink::proto::QosReliabilityPolicy;

class QosProfileConf
{
public:
    QosProfileConf();
    virtual ~QosProfileConf();

    static QosProfile CreateQosProfile(const QosHistoryPolicy& history,
                                       uint32_t depth, uint32_t mps,
                                       const QosReliabilityPolicy& reliability,
                                       const QosDurabilityPolicy& durability);

    static const uint32_t QOS_HISTORY_DEPTH_SYSTEM_DEFAULT;
    static const uint32_t QOS_MPS_SYSTEM_DEFAULT;

    static const QosProfile QOS_PROFILE_DEFAULT;
    static const QosProfile QOS_PROFILE_SENSOR_DATA;
    static const QosProfile QOS_PROFILE_PARAMETERS;
    static const QosProfile QOS_PROFILE_SERVICES_DEFAULT;
    static const QosProfile QOS_PROFILE_PARAM_EVENT;
    static const QosProfile QOS_PROFILE_SYSTEM_DEFAULT;
    static const QosProfile QOS_PROFILE_TF_STATIC;
    static const QosProfile QOS_PROFILE_TOPO_CHANGE;
};

}  // namespace transport
}  // namespace autolink
