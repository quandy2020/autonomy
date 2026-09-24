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
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

extern char** environ;

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

/**
 * @brief Run @p cmd and stop it when @p timeout_sec elapses.
 *
 * bluetoothctl 5.64 ignores a successful command completion when --timeout
 * is set, so pair/connect must be launched without that flag and killed here.
 */
std::pair<int, std::string> RunTimed(const std::string& cmd, int timeout_sec) {
  if (timeout_sec < 1) {
    timeout_sec = 1;
  }
  int fds[2];
  if (pipe(fds) != 0) {
    return {127, "pipe failed"};
  }
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, fds[1], STDERR_FILENO);
  posix_spawn_file_actions_addclose(&actions, fds[0]);
  posix_spawn_file_actions_addclose(&actions, fds[1]);

  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attr, 0);

  char* argv[] = {const_cast<char*>("/bin/sh"), const_cast<char*>("-c"),
                  const_cast<char*>(cmd.c_str()), nullptr};
  pid_t pid = -1;
  const int spawned =
      posix_spawn(&pid, "/bin/sh", &actions, &attr, argv, environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attr);
  close(fds[1]);
  if (spawned != 0) {
    close(fds[0]);
    return {127, std::string("posix_spawn failed: ") + std::strerror(spawned)};
  }
  fcntl(fds[0], F_SETFL, O_NONBLOCK);

  std::string output;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
  int code = -1;
  bool child_done = false;
  bool timed_out = false;
  while (!child_done) {
    int wait_ms = 200;
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      if (!timed_out) {
        timed_out = true;
        kill(-pid, SIGTERM);
      } else {
        kill(-pid, SIGKILL);
      }
    } else {
      wait_ms = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
              .count());
      if (wait_ms > 200) {
        wait_ms = 200;
      }
      if (wait_ms < 1) {
        wait_ms = 1;
      }
    }
    pollfd pfd{};
    pfd.fd = fds[0];
    pfd.events = POLLIN;
    poll(&pfd, 1, wait_ms);
    if ((pfd.revents & (POLLIN | POLLHUP)) != 0) {
      std::array<char, 512> buf{};
      while (true) {
        const ssize_t n = read(fds[0], buf.data(), buf.size());
        if (n > 0) {
          output.append(buf.data(), static_cast<std::size_t>(n));
          continue;
        }
        break;
      }
    }
    int status = 0;
    if (waitpid(pid, &status, WNOHANG) == pid) {
      child_done = true;
      if (WIFEXITED(status)) {
        code = WEXITSTATUS(status);
      } else if (WIFSIGNALED(status)) {
        code = 128 + WTERMSIG(status);
      }
    }
  }
  std::array<char, 512> buf{};
  while (true) {
    const ssize_t n = read(fds[0], buf.data(), buf.size());
    if (n > 0) {
      output.append(buf.data(), static_cast<std::size_t>(n));
      continue;
    }
    break;
  }
  close(fds[0]);
  if (timed_out && code < 0) {
    code = 124;
  }
  return {code, output};
}

std::pair<int, std::string> RunCommand(const std::string& cmd) {
  return RunTimed(cmd, 20);
}

/** @brief bluetoothctl scan on stays up until the process is killed. */
pid_t StartQuiet(const std::string& cmd) {
  const int devnull = open("/dev/null", O_RDWR);
  if (devnull < 0) {
    return -1;
  }
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, devnull, STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, devnull, STDERR_FILENO);
  posix_spawn_file_actions_addclose(&actions, devnull);

  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attr, 0);

  char* argv[] = {const_cast<char*>("/bin/sh"), const_cast<char*>("-c"),
                  const_cast<char*>(cmd.c_str()), nullptr};
  pid_t pid = -1;
  const int spawned =
      posix_spawn(&pid, "/bin/sh", &actions, &attr, argv, environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attr);
  close(devnull);
  if (spawned != 0) {
    return -1;
  }
  return pid;
}

void StopQuiet(pid_t pid) {
  if (pid <= 0) {
    return;
  }
  kill(-pid, SIGTERM);
  for (int i = 0; i < 20; ++i) {
    if (waitpid(pid, nullptr, WNOHANG) == pid) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  kill(-pid, SIGKILL);
  waitpid(pid, nullptr, 0);
}

/**
 * @brief Stop a scan client, then ask BlueZ to stop discovery.
 *
 * A live `scan on` client keeps Discovering: yes, and a second bluetoothctl
 * `scan off` then fails. Pairing while discovery is on drops the DualSense.
 */
void StopDiscovery(pid_t scan_pid) {
  StopQuiet(scan_pid);
  RunTimed("bluetoothctl scan off", 2);
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

bool BluetoothInfoContains(const std::string& mac, const char* needle) {
  const auto info = RunCommand("bluetoothctl info " + mac);
  return info.second.find(needle) != std::string::npos;
}

bool BluetoothInfoConnected(const std::string& mac) {
  return BluetoothInfoContains(mac, "Connected: yes");
}

bool BluetoothInfoBonded(const std::string& mac) {
  return BluetoothInfoContains(mac, "Bonded: yes");
}

/** Last "Key: yes/no" wins. A later "Connected: no" overrides an earlier yes. */
bool LastPropertyYes(const std::string& text, const char* key) {
  const std::string yes = std::string(key) + " yes";
  const std::string no = std::string(key) + " no";
  const auto yes_at = text.rfind(yes);
  const auto no_at = text.rfind(no);
  if (yes_at == std::string::npos) {
    return false;
  }
  return no_at == std::string::npos || yes_at > no_at;
}

bool PairOutputBonded(const std::string& text) {
  return text.find("Pairing successful") != std::string::npos ||
         LastPropertyYes(text, "Bonded:");
}

std::string BriefBluetoothFailure(const std::string& text) {
  std::string last;
  std::istringstream iss(text);
  std::string line;
  while (std::getline(iss, line)) {
    if (line.find("Failed") != std::string::npos ||
        line.find("not available") != std::string::npos ||
        line.find("Host is down") != std::string::npos ||
        line.find("Error") != std::string::npos) {
      last = line;
    }
  }
  if (!last.empty()) {
    return last;
  }
  return "no result";
}

/**
 * @brief Run one bluetoothctl command with an agent, and wait until it finishes.
 *
 * Do not pass bluetoothctl --timeout. On BlueZ 5.64 that flag disables the
 * early quit after "Pairing successful", so the agent stays up, discovery
 * events keep flowing, and the controller drops the link before HID binds.
 */
std::pair<int, std::string> Bluetoothctl(int timeout_sec,
                                        const std::string& args) {
  return RunTimed("bluetoothctl --agent NoInputNoOutput " + args, timeout_sec);
}

/** Drop a half-paired controller so the next pair can store a link key. */
void RemoveUnbonded(const std::string& mac) {
  AWARN << "joy: " << mac
        << " has no link key (Bonded: no); removing it so HID can bind";
  RunCommand("bluetoothctl disconnect " + mac);
  RunCommand("bluetoothctl remove " + mac);
}

BtDevice FindListedDualSense() {
  const auto devices = ListDevices("bluetoothctl devices");
  for (const BtDevice& device : devices) {
    if (IsDualSenseName(device.name)) {
      return device;
    }
  }
  return {};
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

bool ConnectDualSenseBluetooth(int timeout_sec, std::string* device_path) {
  if (timeout_sec < 1) {
    timeout_sec = 1;
  }
  if (!CommandExists("bluetoothctl")) {
    AERROR << "joy: bluetoothctl not found (install bluez)";
    return false;
  }

  Bluetoothctl(5, "pairable on");
  const auto power = Bluetoothctl(5, "power on");
  if (power.first != 0 && power.second.find("succeeded") == std::string::npos) {
    AERROR << "joy: bluetoothctl power on failed (exit=" << power.first
           << ")\n"
           << power.second;
    return false;
  }

  BtDevice target = FindListedDualSense();
  // Paired without a link key makes BlueZ reject the HID connection. A device
  // that is only discovered (not paired) must be kept so pair can run.
  if (!target.mac.empty() && !BluetoothInfoBonded(target.mac) &&
      BluetoothInfoContains(target.mac, "Paired: yes")) {
    RemoveUnbonded(target.mac);
    target = {};
  }
  pid_t scan_pid = -1;
  if (target.mac.empty()) {
    AINFO << "joy: no DualSense in the known list, scanning " << timeout_sec
          << "s (hold Create + PS until the light bar flashes)";
    scan_pid = StartQuiet("bluetoothctl scan on");
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      target = FindListedDualSense();
      if (!target.mac.empty()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    StopDiscovery(scan_pid);
  }

  if (target.mac.empty()) {
    AERROR << "joy: DualSense not found. Enter pairing mode (Create + PS).";
    return false;
  }

  if (!BluetoothInfoBonded(target.mac)) {
    AINFO << "joy: bonding " << target.mac << " (" << target.name << ")";
    const auto paired = Bluetoothctl(timeout_sec, "pair " + target.mac);
    if (!PairOutputBonded(paired.second) && !BluetoothInfoBonded(target.mac)) {
      AERROR << "joy: pair did not bond " << target.mac
             << " (hold Create + PS until the light bar flashes): "
             << BriefBluetoothFailure(paired.second);
      return false;
    }
  } else {
    AINFO << "joy: " << target.mac << " already bonded, connecting";
  }
  {
    const auto trusted = Bluetoothctl(10, "trust " + target.mac);
    if (trusted.first != 0 &&
        trusted.second.find("succeeded") == std::string::npos &&
        !BluetoothInfoContains(target.mac, "Trusted: yes")) {
      AERROR << "joy: trust failed: " << BriefBluetoothFailure(trusted.second);
      return false;
    }
  }
  {
    // Connect as soon as the bond exists. The controller sleeps within a few
    // seconds of pairing and then answers with Host is down until PS is pressed.
    const auto connected = Bluetoothctl(12, "connect " + target.mac);
    const bool up =
        connected.second.find("Connection successful") != std::string::npos ||
        BluetoothInfoConnected(target.mac);
    if (!up) {
      if (connected.second.find("Host is down") != std::string::npos ||
          connected.second.find("br-connection-create-socket") !=
              std::string::npos) {
        AERROR << "joy: " << target.mac
               << " is bonded but not answering; short-press PS to wake it";
      } else {
        AERROR << "joy: connect failed: "
               << BriefBluetoothFailure(connected.second);
      }
      return false;
    }
  }
  if (!BluetoothInfoConnected(target.mac)) {
    AERROR << "joy: " << target.mac << " is not connected";
    return false;
  }

  TryLoadHidPlaystation();
  if (!WaitForJoystick(8, true)) {
    AWARN << "joy: Bluetooth connected but /dev/input/js* is not ready yet";
    return false;
  }
  if (device_path != nullptr) {
    const std::string dual = FindDualSenseJoystickFromProc();
    if (!dual.empty()) {
      *device_path = dual;
    } else {
      const auto nodes = ListJoystickNodes();
      if (!nodes.empty()) {
        *device_path = nodes.front();
      }
    }
  }
  AINFO << "joy: DualSense Bluetooth ready"
        << (device_path && !device_path->empty() ? " device=" + *device_path
                                                 : "");
  return true;
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
  RemoveMatchingPaired();

  AINFO << "pair-joy: scanning…";
  const pid_t scan_pid = StartQuiet("bluetoothctl scan on");

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

  StopDiscovery(scan_pid);

  if (target.mac.empty()) {
    AERROR << "pair-joy: no DualSense / Wireless Controller found within "
           << timeout_sec
           << "s. Ensure pairing mode (Create+PS) and adapter is powered.";
    return false;
  }

  AINFO << "pair-joy: found " << target.mac << " (" << target.name << ")";

  {
    const auto r = Bluetoothctl(timeout_sec, "pair " + target.mac);
    if (!PairOutputBonded(r.second) && !BluetoothInfoBonded(target.mac)) {
      AERROR << "pair-joy: pair did not bond " << target.mac << ": "
             << BriefBluetoothFailure(r.second);
      return false;
    }
  }
  {
    const auto r = Bluetoothctl(10, "trust " + target.mac);
    if (r.first != 0 && r.second.find("succeeded") == std::string::npos) {
      AERROR << "pair-joy: trust failed: " << r.second;
      return false;
    }
  }
  {
    const auto r = Bluetoothctl(20, "connect " + target.mac);
    if (r.first != 0 && !BluetoothInfoConnected(target.mac)) {
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
