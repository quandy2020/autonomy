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

/**
 * @brief Clear soft rfkill and turn the adapter on before pair/scan.
 * @return true when bluetoothctl reports Powered or power on succeeds.
 */
bool EnsureBluetoothAdapterPowered() {
  if (CommandExists("rfkill")) {
    // Soft-block is the usual reason for org.bluez.Error.Failed in Docker.
    const auto unblock = RunCommand("rfkill unblock bluetooth");
    if (unblock.first != 0) {
      AWARN << "joy: rfkill unblock bluetooth failed (exit=" << unblock.first
            << "); continuing with power on\n"
            << unblock.second;
    } else {
      // BlueZ needs a moment after unblock before power on.
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
  }

  auto power = RunCommand("bluetoothctl power on");
  if (power.first == 0 ||
      power.second.find("succeeded") != std::string::npos) {
    // Pairable defaults to no after some resets; DualSense then reports
    // "Pairing successful" without storing Paired/Bonded and vanishes.
    RunCommand("bluetoothctl pairable on");
    return true;
  }

  // Busy / race: off then on once.
  RunCommand("bluetoothctl power off");
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  power = RunCommand("bluetoothctl power on");
  if (power.first == 0 ||
      power.second.find("succeeded") != std::string::npos) {
    RunCommand("bluetoothctl pairable on");
    return true;
  }

  AERROR << "joy: bluetoothctl power on failed (exit=" << power.first << ")\n"
         << power.second
         << "Hint: adapter soft-blocked? On the host run: "
            "rfkill unblock bluetooth && bluetoothctl power on "
            "(container needs /dev/rfkill, usually via privileged + /dev).";
  return false;
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

bool HidPlaystationLoaded() {
  return access("/sys/module/hid_playstation", F_OK) == 0;
}

/**
 * @brief Load hid_playstation (+ joydev) so DualSense exposes /dev/input/js*.
 *
 * Must run on the host kernel. In Docker this needs a host /lib/modules mount
 * (run_autonomy.py) and privileged; otherwise load on the host with sudo.
 */
bool TryLoadHidPlaystation() {
  if (!CommandExists("modprobe")) {
    AWARN << "pair-joy: modprobe not found; cannot load hid_playstation";
    return HidPlaystationLoaded();
  }
  // BlueZ userspace HID + joydev mapping + DualSense driver.
  RunCommand("modprobe uhid");
  RunCommand("modprobe joydev");
  if (HidPlaystationLoaded()) {
    return true;
  }
  const auto r = RunCommand("modprobe hid_playstation");
  if (r.first == 0 || HidPlaystationLoaded()) {
    AINFO << "pair-joy: hid_playstation module loaded";
    return true;
  }
  AWARN << "pair-joy: modprobe hid_playstation exit=" << r.first << "\n"
        << r.second
        << "Load on the HOST (container has no kernel modules unless "
           "/lib/modules is mounted):\n"
           "  sudo modprobe hid_playstation uhid joydev";
  return false;
}

bool BluetoothInfoContains(const std::string& mac, const char* needle) {
  const auto info = RunCommand("bluetoothctl info " + mac);
  return info.second.find(needle) != std::string::npos;
}

bool BluetoothInfoConnected(const std::string& mac) {
  return BluetoothInfoContains(mac, "Connected: yes");
}

bool BluetoothInfoPaired(const std::string& mac) {
  return BluetoothInfoContains(mac, "Paired: yes");
}

std::string BluezDevicePath(const std::string& mac) {
  std::string path = mac;
  for (char& c : path) {
    if (c == ':') {
      c = '_';
    }
  }
  return "/org/bluez/hci0/dev_" + path;
}

/**
 * @brief Whether BlueZ stored a durable link key for @p mac (Bonded).
 *
 * Prefer busctl Bonded. When the property is missing, fall back to Paired.
 * Do not treat an explicit Bonded=false as success.
 */
bool BluetoothInfoBonded(const std::string& mac) {
  if (CommandExists("busctl")) {
    const auto prop = RunCommand(
        "busctl get-property org.bluez " + BluezDevicePath(mac) +
        " org.bluez.Device1 Bonded 2>/dev/null");
    // busctl prints "b true" / "b false" for boolean properties.
    if (prop.second.find("b true") != std::string::npos) {
      return true;
    }
    if (prop.second.find("b false") != std::string::npos) {
      return false;
    }
  }
  const auto info = RunCommand("bluetoothctl info " + mac);
  if (info.second.find("Bonded: yes") != std::string::npos) {
    return true;
  }
  if (info.second.find("Bonded: no") != std::string::npos) {
    return false;
  }
  // No Bonded field (older bluetoothctl text): Paired implies a stored key.
  return info.second.find("Paired: yes") != std::string::npos;
}

/** @brief Pair or Bonded — either means BlueZ kept the device after pair. */
bool BluetoothInfoPairedOrBonded(const std::string& mac) {
  return BluetoothInfoBonded(mac) || BluetoothInfoPaired(mac);
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

bool PairingSuccessfulOutput(const std::string& text) {
  return text.find("Pairing successful") != std::string::npos;
}

bool PairOutputBonded(const std::string& text) {
  return PairingSuccessfulOutput(text) || LastPropertyYes(text, "Bonded:");
}

/** @brief Poll until Bonded (link key) or timeout. */
bool WaitBonded(const std::string& mac, int wait_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(wait_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (BluetoothInfoBonded(mac)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return BluetoothInfoBonded(mac);
}

/**
 * @brief Ask BlueZ to open the HID profile (creates uhid / hid device).
 * @return true when the busctl call reports success.
 */
bool ConnectHidProfile(const std::string& mac) {
  if (!CommandExists("busctl")) {
    return false;
  }
  const std::string cmd =
      "busctl call org.bluez " + BluezDevicePath(mac) +
      " org.bluez.Device1 ConnectProfile s "
      "00001124-0000-1000-8000-00805f9b34fb";
  const auto r = RunCommand(cmd);
  if (r.first == 0) {
    AINFO << "pair-joy: HID profile connect requested for " << mac;
    return true;
  }
  AWARN << "pair-joy: ConnectProfile(HID) failed: " << r.second;
  return false;
}

/**
 * @brief Request HID and wait for js* without host-initiated disconnect.
 *
 * DualSense ReconnectMode is device-driven; `bluetoothctl disconnect` then
 * connect often yields br-connection-create-socket and drops a half-pair.
 */
bool WaitForHidJoystick(const std::string& mac, int wait_sec) {
  ConnectHidProfile(mac);
  if (WaitForJoystick(std::max(8, wait_sec / 2), true)) {
    return true;
  }
  // Still connected: ask HID again; never disconnect to "retry".
  if (BluetoothInfoConnected(mac)) {
    ConnectHidProfile(mac);
  } else {
    AWARN << "pair-joy: ACL dropped before HID; short-press PS, then connect";
    RunTimed("bluetoothctl --agent NoInputNoOutput connect " + mac, 15);
    ConnectHidProfile(mac);
  }
  return WaitForJoystick(wait_sec, true);
}

std::string BriefBluetoothFailure(const std::string& text) {
  std::string last;
  std::istringstream iss(text);
  std::string line;
  while (std::getline(iss, line)) {
    if (line.find("Failed") != std::string::npos ||
        line.find("not available") != std::string::npos ||
        line.find("Host is down") != std::string::npos ||
        line.find("Authentication") != std::string::npos ||
        line.find("Cancelled") != std::string::npos ||
        line.find("Error") != std::string::npos) {
      last = line;
    }
  }
  if (!last.empty()) {
    return last;
  }
  // Keep a short non-empty snippet for debugging empty/"no result" cases.
  std::string trimmed = text;
  while (!trimmed.empty() &&
         std::isspace(static_cast<unsigned char>(trimmed.back()))) {
    trimmed.pop_back();
  }
  if (!trimmed.empty()) {
    const auto nl = trimmed.find_last_of('\n');
    if (nl != std::string::npos) {
      trimmed = trimmed.substr(nl + 1);
    }
    if (trimmed.size() > 160) {
      trimmed.resize(160);
    }
    return trimmed;
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

/**
 * @brief After "Pairing successful", trust+connect before the DualSense sleeps.
 *
 * Waiting many seconds for Bonded alone lets the controller drop; BlueZ then
 * deletes the device (Paired=no) and HID never appears.
 */
bool FinalizePairTrustConnect(const std::string& mac) {
  AINFO << "pair-joy: Pairing successful — trust+connect immediately " << mac;
  {
    const auto trust = RunTimed("bluetoothctl trust " + mac, 8);
    if (trust.first != 0 &&
        trust.second.find("succeeded") == std::string::npos &&
        !BluetoothInfoContains(mac, "Trusted: yes")) {
      AWARN << "pair-joy: trust: " << BriefBluetoothFailure(trust.second);
    }
  }
  {
    const auto conn = RunTimed("bluetoothctl connect " + mac, 20);
    if (conn.first != 0 && !BluetoothInfoConnected(mac)) {
      AWARN << "pair-joy: connect after pair: "
            << BriefBluetoothFailure(conn.second)
            << " (short-press PS if the light went out)";
    }
  }
  if (WaitBonded(mac, 5000)) {
    return true;
  }
  if (BluetoothInfoPairedOrBonded(mac)) {
    AWARN << "pair-joy: Bonded not set yet; Paired="
          << (BluetoothInfoPaired(mac) ? "yes" : "no")
          << " Connected=" << (BluetoothInfoConnected(mac) ? "yes" : "no");
    return true;
  }
  return BluetoothInfoConnected(mac);
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

  if (!EnsureBluetoothAdapterPowered()) {
    return false;
  }
  TryLoadHidPlaystation();
  Bluetoothctl(5, "pairable on");

  BtDevice target = FindListedDualSense();
  const pid_t agent_pid =
      StartQuiet("bluetoothctl --agent NoInputNoOutput");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  pid_t scan_pid = -1;
  const bool need_scan =
      target.mac.empty() ||
      !(BluetoothInfoBonded(target.mac) || BluetoothInfoPaired(target.mac));
  if (need_scan) {
    AINFO << "joy: scanning for DualSense (hold Create + PS until the light "
             "bar flashes)";
    scan_pid = StartQuiet("bluetoothctl scan on");
  }
  if (target.mac.empty()) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      target = FindListedDualSense();
      if (!target.mac.empty()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  if (target.mac.empty()) {
    StopDiscovery(scan_pid);
    StopQuiet(agent_pid);
    AERROR << "joy: DualSense not found. Enter pairing mode (Create + PS).";
    return false;
  }

  // Stop discovery before pair so Bonded can settle.
  StopDiscovery(scan_pid);
  scan_pid = -1;
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  if (!BluetoothInfoBonded(target.mac) && !BluetoothInfoPaired(target.mac)) {
    AINFO << "joy: bonding " << target.mac << " (" << target.name << ")";
    const auto paired =
        RunTimed("bluetoothctl pair " + target.mac, timeout_sec);
    bool ok = false;
    if (PairingSuccessfulOutput(paired.second) ||
        WaitBonded(target.mac, 2000) || BluetoothInfoPaired(target.mac)) {
      ok = FinalizePairTrustConnect(target.mac);
    } else {
      ok = WaitBonded(target.mac, 8000);
    }
    if (!ok) {
      StopQuiet(agent_pid);
      AERROR << "joy: pair did not stick for " << target.mac
             << " (hold Create + PS until the light bar flashes): "
             << BriefBluetoothFailure(paired.second);
      RemoveUnbonded(target.mac);
      return false;
    }
  } else {
    AINFO << "joy: " << target.mac << " already paired/bonded, connecting";
    RunTimed("bluetoothctl trust " + target.mac, 10);
    const auto connected = RunTimed("bluetoothctl connect " + target.mac, 12);
    const bool up =
        connected.second.find("Connection successful") != std::string::npos ||
        BluetoothInfoConnected(target.mac);
    if (!up) {
      StopQuiet(agent_pid);
      if (connected.second.find("Host is down") != std::string::npos ||
          connected.second.find("br-connection-create-socket") !=
              std::string::npos) {
        AERROR << "joy: " << target.mac
               << " is paired but not answering; short-press PS to wake it";
      } else {
        AERROR << "joy: connect failed: "
               << BriefBluetoothFailure(connected.second);
      }
      return false;
    }
  }
  if (!BluetoothInfoConnected(target.mac)) {
    StopQuiet(agent_pid);
    AERROR << "joy: " << target.mac << " is not connected";
    return false;
  }

  TryLoadHidPlaystation();
  const bool hid_ok = WaitForHidJoystick(target.mac, 20);
  StopQuiet(agent_pid);
  if (!hid_ok) {
    AWARN << "joy: Bluetooth connected but /dev/input/js* is not ready yet "
             "(need Bonded: yes + hid_playstation/uhid; short-press PS if asleep)";
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

  if (!EnsureBluetoothAdapterPowered()) {
    return false;
  }
  RemoveMatchingPaired();

  // Long-lived agent so Pair can store a link key (Bonded) before the process
  // exits. Short-lived `bluetoothctl --agent … pair` often leaves Paired-only.
  const pid_t agent_pid =
      StartQuiet("bluetoothctl --agent NoInputNoOutput");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Discover first; stop scan before pair so the bond can settle (pairing
  // under active discovery often yields Paired without Bonded).
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

  if (target.mac.empty()) {
    StopDiscovery(scan_pid);
    StopQuiet(agent_pid);
    AERROR << "pair-joy: no DualSense / Wireless Controller found within "
           << timeout_sec
           << "s. Ensure pairing mode (Create+PS) and adapter is powered.";
    return false;
  }

  AINFO << "pair-joy: found " << target.mac << " (" << target.name << ")";
  StopDiscovery(scan_pid);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  // Load before connect so the HID stack binds DualSense to playstation driver.
  if (!TryLoadHidPlaystation()) {
    StopQuiet(agent_pid);
    AERROR << "pair-joy: hid_playstation is required for /dev/input/js*. "
              "On the host: sudo modprobe hid_playstation && "
              "sudo modprobe joydev  (then recreate SpaceHero with "
              "/lib/modules mounted, or load modules on the host).";
    return false;
  }

  std::string last_pair_out;
  bool pair_ok = false;
  for (int attempt = 1; attempt <= 3 && !pair_ok; ++attempt) {
    AINFO << "pair-joy: bonding attempt " << attempt << "/3 for " << target.mac;
    const int pair_budget = std::max(20, timeout_sec / 2);
    // Agent already registered; do not start a second --agent for pair.
    const auto r =
        RunTimed("bluetoothctl pair " + target.mac, pair_budget);
    last_pair_out = r.second;

    // Pairing successful → trust+connect immediately (DualSense sleeps fast).
    if (PairingSuccessfulOutput(r.second) || PairOutputBonded(r.second) ||
        WaitBonded(target.mac, 1500)) {
      pair_ok = FinalizePairTrustConnect(target.mac);
      if (pair_ok) {
        break;
      }
      AWARN << "pair-joy: pair reported success but device did not stick "
               "(Paired="
            << (BluetoothInfoPaired(target.mac) ? "yes" : "no")
            << " Bonded=" << (BluetoothInfoBonded(target.mac) ? "yes" : "no")
            << "). Re-enter Create+PS and retry.";
      // Second pair() often wipes state — only retry if device is gone.
      if (BluetoothInfoPairedOrBonded(target.mac) ||
          BluetoothInfoConnected(target.mac)) {
        break;
      }
      continue;
    }

    pair_ok = WaitBonded(target.mac, 4000);
    if (pair_ok) {
      pair_ok = FinalizePairTrustConnect(target.mac);
      break;
    }
    const std::string why = BriefBluetoothFailure(r.second);
    AWARN << "pair-joy: bond attempt " << attempt << " failed: " << why;
    if (why.find("not available") != std::string::npos) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }
  }

  if (!pair_ok) {
    StopQuiet(agent_pid);
    AERROR << "pair-joy: pair did not stick for " << target.mac << ": "
           << BriefBluetoothFailure(last_pair_out)
           << " Ensure adapter Pairable: yes, hold Create+PS until the light "
              "bar flashes, then retry.";
    return false;
  }
  AINFO << "pair-joy: paired/bonded " << target.mac
        << " Bonded=" << (BluetoothInfoBonded(target.mac) ? "yes" : "no")
        << " Paired=" << (BluetoothInfoPaired(target.mac) ? "yes" : "no")
        << " Connected="
        << (BluetoothInfoConnected(target.mac) ? "yes" : "no");

  if (!BluetoothInfoConnected(target.mac)) {
    const auto r = RunTimed("bluetoothctl connect " + target.mac, 20);
    if (r.first != 0 && !BluetoothInfoConnected(target.mac)) {
      StopQuiet(agent_pid);
      AERROR << "pair-joy: connect failed: " << r.second
             << " Short-press PS to wake the controller, then retry.";
      return false;
    }
  }

  AINFO << "pair-joy: trusted + connected " << target.mac;
  const bool hid_ok = WaitForHidJoystick(target.mac, 20);
  StopQuiet(agent_pid);
  if (!hid_ok) {
    AERROR << "pair-joy: connected but no /dev/input/js* after wait. "
              "Check: `bluetoothctl info "
           << target.mac
           << "` (Bonded/Paired); host modules "
              "`hid_playstation uhid joydev`; short-press PS if asleep. "
              "Do not disconnect/reconnect from the host while waking.";
    return false;
  }
  AINFO << "pair-joy: done. Enable joy in YAML (device auto-detects js*).";
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
