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
 * @file joy_pair.cpp
 * @brief DualSense Bluetooth re-pair and USB/driver wait via --pair-joy.
 */

#include "autodriver/joy/joy_pair.hpp"

#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <utility>
#include <vector>

#include "autolink/common/log.hpp"

namespace autodriver {
namespace joy {
namespace {

struct BtDevice {
  std::string mac;
  std::string name;
};

std::string ToLower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

bool IsDualSenseName(const std::string& name) {
  const std::string lower = ToLower(name);
  return lower.find("dualsense") != std::string::npos ||
         lower.find("wireless controller") != std::string::npos;
}

/**
 * @brief Sony DualSense USB/BT product IDs commonly seen on Linux.
 *
 * 0x0ce6 DualSense, 0x0df2 DualSense Edge; 0x05c4/0x09cc are DualShock 4
 * (accepted as "Wireless Controller" family for teleop).
 */
bool IsSonyControllerIds(const std::string& vendor_hex,
                         const std::string& product_hex) {
  const std::string v = ToLower(vendor_hex);
  const std::string p = ToLower(product_hex);
  if (v != "054c") {
    return false;
  }
  return p == "0ce6" || p == "0df2" || p == "05c4" || p == "09cc";
}

std::pair<int, std::string> RunCommand(const std::string& cmd) {
  FILE* pipe = popen((cmd + " 2>&1").c_str(), "r");
  if (pipe == nullptr) {
    return {127, "popen failed for: " + cmd};
  }
  std::string output;
  std::array<char, 512> buf{};
  while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) {
    output.append(buf.data());
  }
  const int status = pclose(pipe);
  int code = -1;
  if (WIFEXITED(status)) {
    code = WEXITSTATUS(status);
  }
  return {code, output};
}

bool CommandExists(const char* name) {
  const auto r = RunCommand(std::string("command -v ") + name);
  return r.first == 0 && !r.second.empty();
}

std::vector<BtDevice> ParseDevices(const std::string& text) {
  std::vector<BtDevice> out;
  std::istringstream iss(text);
  std::string line;
  while (std::getline(iss, line)) {
    if (line.rfind("Device ", 0) != 0) {
      continue;
    }
    const std::size_t mac_start = 7;
    if (line.size() < mac_start + 17) {
      continue;
    }
    const std::string mac = line.substr(mac_start, 17);
    std::string name;
    if (line.size() > mac_start + 18) {
      name = line.substr(mac_start + 18);
      while (!name.empty() &&
             std::isspace(static_cast<unsigned char>(name.front()))) {
        name.erase(name.begin());
      }
      while (!name.empty() &&
             std::isspace(static_cast<unsigned char>(name.back()))) {
        name.pop_back();
      }
    }
    if (mac.size() == 17 && mac[2] == ':') {
      out.push_back({mac, name});
    }
  }
  return out;
}

std::vector<BtDevice> ListDevices(const char* list_cmd) {
  const auto r = RunCommand(list_cmd);
  return ParseDevices(r.second);
}

void RemoveMatchingPaired() {
  const auto paired = ListDevices("bluetoothctl devices Paired");
  for (const BtDevice& d : paired) {
    if (!IsDualSenseName(d.name)) {
      continue;
    }
    AINFO << "pair-joy: removing previous binding " << d.mac << " (" << d.name
          << ")";
    const auto r = RunCommand("bluetoothctl remove " + d.mac);
    if (r.first != 0) {
      AWARN << "pair-joy: remove " << d.mac << " exit=" << r.first << " "
            << r.second;
    }
  }
}

std::vector<std::string> ListJoystickNodes() {
  std::vector<std::string> nodes;
  DIR* dir = opendir("/dev/input");
  if (dir == nullptr) {
    return nodes;
  }
  while (dirent* ent = readdir(dir)) {
    const std::string name = ent->d_name;
    if (name.rfind("js", 0) == 0 && name.size() > 2 &&
        std::isdigit(static_cast<unsigned char>(name[2]))) {
      nodes.push_back("/dev/input/" + name);
    }
  }
  closedir(dir);
  return nodes;
}

/**
 * @brief Best-effort: find jsN handlers whose /proc block looks like DualSense.
 *
 * @return Absolute path of a matching js node, or empty.
 */
std::string FindDualSenseJoystickFromProc() {
  std::ifstream in("/proc/bus/input/devices");
  if (!in) {
    return {};
  }
  std::string line;
  std::string name;
  std::string vendor;
  std::string product;
  std::string handlers;
  auto flush = [&]() -> std::string {
    const bool name_ok = IsDualSenseName(name);
    const bool id_ok = IsSonyControllerIds(vendor, product);
    std::string js_path;
    if (name_ok || id_ok) {
      // handlers=kbd eventX js0
      std::istringstream hs(handlers);
      std::string tok;
      while (hs >> tok) {
        if (tok.rfind("js", 0) == 0 && tok.size() > 2 &&
            std::isdigit(static_cast<unsigned char>(tok[2]))) {
          js_path = "/dev/input/" + tok;
          break;
        }
      }
    }
    name.clear();
    vendor.clear();
    product.clear();
    handlers.clear();
    return js_path;
  };

  while (std::getline(in, line)) {
    if (line.empty()) {
      const std::string path = flush();
      if (!path.empty()) {
        return path;
      }
      continue;
    }
    if (line.rfind("N: Name=", 0) == 0) {
      name = line.substr(8);
      if (!name.empty() && name.front() == '"') {
        name.erase(name.begin());
      }
      if (!name.empty() && name.back() == '"') {
        name.pop_back();
      }
    } else if (line.rfind("I: ", 0) == 0) {
      // I: Bus=0003 Vendor=054c Product=0ce6 Version=0100
      std::istringstream iss(line.substr(3));
      std::string field;
      while (iss >> field) {
        if (field.rfind("Vendor=", 0) == 0) {
          vendor = field.substr(7);
        } else if (field.rfind("Product=", 0) == 0) {
          product = field.substr(8);
        }
      }
    } else if (line.rfind("H: Handlers=", 0) == 0) {
      handlers = line.substr(12);
    }
  }
  return flush();
}

bool WaitForJoystick(int wait_sec, bool prefer_dualsense) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(wait_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (prefer_dualsense) {
      const std::string dual = FindDualSenseJoystickFromProc();
      if (!dual.empty()) {
        AINFO << "pair-joy: DualSense joystick ready: " << dual;
        return true;
      }
    }
    const auto nodes = ListJoystickNodes();
    if (!nodes.empty()) {
      AINFO << "pair-joy: joystick node ready: " << nodes.front();
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return false;
}

void TryLoadHidPlaystation() {
  // Best-effort; may fail without root — USB hotplug often loads it already.
  if (!CommandExists("modprobe")) {
    return;
  }
  const auto r = RunCommand("modprobe hid_playstation");
  if (r.first == 0) {
    AINFO << "pair-joy: hid_playstation module loaded (or already present)";
  } else {
    AWARN << "pair-joy: modprobe hid_playstation exit=" << r.first
          << " (ok if already loaded / need root)\n"
          << r.second;
  }
}

}  // namespace

bool ParseJoyPairMode(const std::string& text, JoyPairMode* mode) {
  if (mode == nullptr) {
    return false;
  }
  const std::string t = ToLower(text);
  if (t == "bluetooth" || t == "bt") {
    *mode = JoyPairMode::kBluetooth;
    return true;
  }
  if (t == "usb" || t == "wired" || t == "driver") {
    *mode = JoyPairMode::kUsb;
    return true;
  }
  return false;
}

const char* JoyPairModeName(JoyPairMode mode) {
  switch (mode) {
    case JoyPairMode::kBluetooth:
      return "bluetooth";
    case JoyPairMode::kUsb:
      return "usb";
  }
  return "unknown";
}

bool PairDualSenseBluetooth(int timeout_sec) {
  if (timeout_sec < 1) {
    timeout_sec = 1;
  }

  if (!CommandExists("bluetoothctl")) {
    AERROR << "pair-joy: bluetoothctl not found (install bluez)";
    return false;
  }

  AINFO << "pair-joy: DualSense Bluetooth re-pair (timeout=" << timeout_sec
        << "s)";
  AINFO << "pair-joy: put the controller in pairing mode: hold Create + PS "
           "until the light bar flashes";

  {
    const auto power = RunCommand("bluetoothctl power on");
    if (power.first != 0) {
      AERROR << "pair-joy: bluetoothctl power on failed (exit=" << power.first
             << "). Need Bluetooth adapter permissions?\n"
             << power.second;
      return false;
    }
  }
  RunCommand("bluetoothctl agent on");
  RunCommand("bluetoothctl default-agent");

  RemoveMatchingPaired();

  AINFO << "pair-joy: scanning…";
  RunCommand("bluetoothctl scan on");

  BtDevice target;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto devices = ListDevices("bluetoothctl devices");
    for (const BtDevice& d : devices) {
      if (IsDualSenseName(d.name)) {
        target = d;
        break;
      }
    }
    if (!target.mac.empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  RunCommand("bluetoothctl scan off");

  if (target.mac.empty()) {
    AERROR << "pair-joy: no DualSense / Wireless Controller found within "
           << timeout_sec
           << "s. Ensure pairing mode (Create+PS) and adapter is powered.";
    return false;
  }

  AINFO << "pair-joy: found " << target.mac << " (" << target.name << ")";

  {
    const auto r = RunCommand("bluetoothctl pair " + target.mac);
    if (r.first != 0) {
      AWARN << "pair-joy: pair exit=" << r.first << " " << r.second;
    }
  }
  {
    const auto r = RunCommand("bluetoothctl trust " + target.mac);
    if (r.first != 0) {
      AERROR << "pair-joy: trust failed: " << r.second;
      return false;
    }
  }
  {
    const auto r = RunCommand("bluetoothctl connect " + target.mac);
    if (r.first != 0) {
      AERROR << "pair-joy: connect failed: " << r.second;
      return false;
    }
  }

  AINFO << "pair-joy: paired + trusted + connected " << target.mac;
  if (!WaitForJoystick(10, true)) {
    AWARN << "pair-joy: connected but no /dev/input/js* yet "
             "(check input group / hid-playstation)";
  }
  AINFO << "pair-joy: done. Enable joy in YAML and set device to the js node.";
  return true;
}

bool PairDualSenseUsb(int timeout_sec) {
  if (timeout_sec < 1) {
    timeout_sec = 1;
  }

  AINFO << "pair-joy: DualSense USB / driver mode (timeout=" << timeout_sec
        << "s)";
  AINFO << "pair-joy: plug the controller in via USB cable now "
           "(hid-playstation will expose /dev/input/js*)";

  TryLoadHidPlaystation();

  if (WaitForJoystick(timeout_sec, true)) {
    AINFO << "pair-joy: USB/driver attach done. Set joy.device in YAML if needed.";
    return true;
  }

  AERROR << "pair-joy: no DualSense joystick within " << timeout_sec
         << "s. Check USB cable, `lsusb | grep Sony`, and "
            "`modprobe hid_playstation` (may need root).";
  return false;
}

bool PairDualSense(JoyPairMode mode, int timeout_sec) {
  switch (mode) {
    case JoyPairMode::kBluetooth:
      return PairDualSenseBluetooth(timeout_sec);
    case JoyPairMode::kUsb:
      return PairDualSenseUsb(timeout_sec);
  }
  AERROR << "pair-joy: unknown mode";
  return false;
}

}  // namespace joy
}  // namespace autodriver
