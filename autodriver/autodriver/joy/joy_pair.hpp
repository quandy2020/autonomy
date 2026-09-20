/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file joy_pair.hpp
 * @brief DualSense pair / wait helpers for CLI --pair-joy (bluetooth + USB).
 */

#ifndef AUTODRIVER_JOY_JOY_PAIR_HPP_
#define AUTODRIVER_JOY_JOY_PAIR_HPP_

#include <string>

namespace autodriver {
namespace joy {

/**
 * @brief How --pair-joy attaches a DualSense controller.
 */
enum class JoyPairMode {
  kBluetooth,  ///< bluetoothctl remove → scan → pair → trust → connect
  kUsb,        ///< USB / hid-playstation: wait for /dev/input/js*
};

/**
 * @brief Parse CLI --pair-mode string.
 *
 * @param[in] text  bluetooth|bt | usb|wired|driver (case-insensitive).
 * @param[out] mode Filled on success.
 * @return true if @p text is a known mode.
 */
bool ParseJoyPairMode(const std::string& text, JoyPairMode* mode);

/**
 * @brief Human-readable name for logging / help.
 */
const char* JoyPairModeName(JoyPairMode mode);

/**
 * @brief Remove existing DualSense Bluetooth bindings, scan, pair, trust, connect.
 *
 * @param[in] timeout_sec Maximum seconds to wait while scanning (clamped >= 1).
 * @return true when pair + trust + connect succeed.
 */
bool PairDualSenseBluetooth(int timeout_sec);

/**
 * @brief USB / kernel-driver mode: ensure hid-playstation and wait for js*.
 *
 * @param[in] timeout_sec Maximum seconds to wait for a joystick node.
 * @return true when a DualSense-like joystick node appears.
 *
 * @details Prints plug-in guidance; best-effort `modprobe hid_playstation`.
 * Identifies DualSense via /proc/bus/input/devices when possible.
 */
bool PairDualSenseUsb(int timeout_sec);

/**
 * @brief Dispatch --pair-joy by mode.
 */
bool PairDualSense(JoyPairMode mode, int timeout_sec);

}  // namespace joy
}  // namespace autodriver

#endif  // AUTODRIVER_JOY_JOY_PAIR_HPP_
