/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

/**
 * @file respeaker_driver.hpp
 * @brief ReSpeaker-style microphone driver factory (skeleton).
 */

#ifndef AUTODRIVER_MICROPHONE_RESPEAKER_DRIVER_HPP_
#define AUTODRIVER_MICROPHONE_RESPEAKER_DRIVER_HPP_

#include <memory>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Create a ReSpeaker / PortAudio capture driver, or nullptr until wired.
 *
 * Planned path: PortAudio (or USB HID) → PCM frames carried as
 * `MicrophoneSample` (Image byte bag placeholder). YAML: `backend: respeaker`.
 *
 * @param[in] id Sensor id (e.g. mic/cabin).
 * @param[in] params Device path / sample rate when implemented.
 * @return Owning raw pointer (or nullptr on failure).
 */
SensorDriver*
CreateRespeakerMicrophoneDriver(
    const SensorId& id, const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_MICROPHONE_RESPEAKER_DRIVER_HPP_
