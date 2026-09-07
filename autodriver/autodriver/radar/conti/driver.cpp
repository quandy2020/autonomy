/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/radar/conti/driver.hpp"

#include "autodriver/radar/backend_register.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace hardware {

std::shared_ptr<SensorDriver> CreateContiRadarDriver(
    const SensorId& id, const DriverParams& /*params*/) {
    AERROR << "Conti radar backend not implemented (id=" << id
           << "); add ProtocolData table under radar/conti/ "
              "(use canbus::CanReceiver + FakeCanClient for tests)";
    return nullptr;
}

}  // namespace hardware
}  // namespace autodriver

REGISTER_RADAR_BACKEND(conti, "conti",
                       autodriver::hardware::CreateContiRadarDriver,
                       "continental");
