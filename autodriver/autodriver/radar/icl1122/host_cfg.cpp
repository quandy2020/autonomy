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
 * @file host_cfg.cpp
 * @brief JoySpace host command builder + EVB1122 bring-up (implementation).
 */

#include "autodriver/radar/icl1122/host_cfg.hpp"

#include <chrono>
#include <fstream>
#include <thread>

#include "autolink/common/log.hpp"

namespace autodriver {
namespace protocol {
namespace {

constexpr std::uint8_t kHdr[4] = {0xFD, 0xFC, 0xFB, 0xFA};
constexpr std::uint8_t kTail[4] = {0x04, 0x03, 0x02, 0x01};
constexpr std::size_t kMaxPacketSize = 1024;

void PushU16Le(std::vector<std::uint8_t>* out, std::uint16_t v) {
  out->push_back(static_cast<std::uint8_t>(v & 0xFF));
  out->push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
}

}  // namespace

std::vector<std::uint8_t> BuildIcl1122HostPacket(
    Icl1122HostCmd cmd, const std::vector<std::uint8_t>& payload) {
  // JoySpace: cmdLen = payload.size() + 2 (cmd id counted in length).
  const std::uint16_t cmd_len =
      static_cast<std::uint16_t>(payload.size() + 2);
  const std::size_t total =
      sizeof(kHdr) + 2 + cmd_len + sizeof(kTail);
  std::vector<std::uint8_t> pkt;
  if (total > kMaxPacketSize || cmd_len == 0) {
    return pkt;
  }
  pkt.reserve(total);
  pkt.insert(pkt.end(), kHdr, kHdr + 4);
  PushU16Le(&pkt, cmd_len);
  PushU16Le(&pkt, static_cast<std::uint16_t>(cmd));
  pkt.insert(pkt.end(), payload.begin(), payload.end());
  pkt.insert(pkt.end(), kTail, kTail + 4);
  return pkt;
}

bool LoadIcl1122RegisterConfig(
    const std::string& path,
    std::vector<std::pair<std::uint16_t, std::uint16_t>>* out_pairs) {
  if (out_pairs == nullptr) {
    return false;
  }
  out_pairs->clear();
  std::ifstream file(path);
  if (!file.is_open()) {
    AERROR << "ICL1122 register config open failed: " << path;
    return false;
  }
  std::string line;
  while (std::getline(file, line)) {
    while (!line.empty() &&
           (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#' || line[0] == ';') {
      continue;
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    try {
      const auto addr = static_cast<std::uint16_t>(
          std::stoul(line.substr(0, eq), nullptr, 16));
      const auto value = static_cast<std::uint16_t>(
          std::stoul(line.substr(eq + 1), nullptr, 16));
      out_pairs->emplace_back(addr, value);
    } catch (...) {
      AWARN << "ICL1122 skip bad register line: " << line;
    }
  }
  return !out_pairs->empty();
}

Icl1122HostConfigurator::Icl1122HostConfigurator(
    common::Stream* stream, std::string register_config_path,
    int response_timeout_ms)
    : stream_(stream),
      register_config_path_(std::move(register_config_path)),
      response_timeout_ms_(response_timeout_ms > 0 ? response_timeout_ms
                                                   : 200) {}

bool Icl1122HostConfigurator::WritePacket(
    Icl1122HostCmd cmd, const std::vector<std::uint8_t>& payload) {
  if (stream_ == nullptr) {
    return false;
  }
  const auto pkt = BuildIcl1122HostPacket(cmd, payload);
  if (pkt.empty()) {
    return false;
  }
  return stream_->Write(pkt.data(), pkt.size());
}

bool Icl1122HostConfigurator::Drain(int timeout_ms) {
  if (stream_ == nullptr) {
    return false;
  }
  std::uint8_t buf[512];
  (void)stream_->Read(buf, sizeof(buf), timeout_ms);
  return true;
}

bool Icl1122HostConfigurator::FlushInput() {
  if (stream_ == nullptr) {
    return false;
  }
  std::uint8_t buf[1024];
  for (int i = 0; i < 8; ++i) {
    const std::size_t n = stream_->Read(buf, sizeof(buf), 50);
    if (n == 0) {
      break;
    }
  }
  return true;
}

bool Icl1122HostConfigurator::WriteAndDrain(
    Icl1122HostCmd cmd, const std::vector<std::uint8_t>& payload) {
  if (!WritePacket(cmd, payload)) {
    return false;
  }
  return Drain(response_timeout_ms_);
}

bool Icl1122HostConfigurator::SendWriteRegBatches(
    const std::vector<std::pair<std::uint16_t, std::uint16_t>>& pairs) {
  // JoySpace WriteReg payload prefix 0x00 0x20, then addr/value LE pairs.
  constexpr std::size_t kMaxPayload =
      kMaxPacketSize - 12;  // hdr+len+cmd+tail ≈ 12
  std::vector<std::uint8_t> buffer;
  buffer.reserve(kMaxPayload);
  buffer.push_back(0x00);
  buffer.push_back(0x20);

  auto flush_batch = [&]() -> bool {
    if (!WriteAndDrain(Icl1122HostCmd::kWriteReg, buffer)) {
      return false;
    }
    buffer.clear();
    buffer.push_back(0x00);
    buffer.push_back(0x20);
    return true;
  };

  for (const auto& p : pairs) {
    if (buffer.size() + 4 > kMaxPayload) {
      if (!flush_batch()) {
        return false;
      }
    }
    PushU16Le(&buffer, p.first);
    PushU16Le(&buffer, p.second);
  }
  return flush_batch();
}

bool Icl1122HostConfigurator::Run() {
  if (stream_ == nullptr) {
    return false;
  }
  std::vector<std::pair<std::uint16_t, std::uint16_t>> regs;
  if (!LoadIcl1122RegisterConfig(register_config_path_, &regs)) {
    return false;
  }
  AINFO << "ICL1122 host bring-up registers=" << regs.size()
        << " file=" << register_config_path_;

  // Mirror Mmwave::StartSensor (24GMmwave/src/Mmwave.cpp).
  const std::vector<std::uint8_t> start_payload{0x02, 0x00};
  for (int i = 0; i < 3; ++i) {
    if (!WritePacket(Icl1122HostCmd::kStartCfg, start_payload)) {
      return false;
    }
  }
  FlushInput();

  if (!WriteAndDrain(Icl1122HostCmd::kGetWhichDemo,
                     std::vector<std::uint8_t>{0x08, 0x00, 0x00, 0x00})) {
    return false;
  }
  if (!WritePacket(Icl1122HostCmd::kStartCfg, start_payload) ||
      !FlushInput()) {
    return false;
  }
  if (!WritePacket(Icl1122HostCmd::kStartCfg, start_payload) ||
      !FlushInput()) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kGetAppRun, {})) {
    return false;
  }
  if (!WritePacket(Icl1122HostCmd::kStartCfg, start_payload) ||
      !FlushInput()) {
    return false;
  }
  if (!WritePacket(Icl1122HostCmd::kStartCfg, start_payload) ||
      !FlushInput()) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kReadVer, {})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kWriteSys,
                     std::vector<std::uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
                                               0x00})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kCascadingMode,
                     std::vector<std::uint8_t>{0x01, 0x00, 0x02, 0x00, 0x01,
                                               0x00})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kWriteSys,
                     std::vector<std::uint8_t>{0x01, 0x00, 0x01, 0x00, 0x00,
                                               0x00})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kWriteSys,
                     std::vector<std::uint8_t>{0x05, 0x00, 0x01, 0x00, 0x00,
                                               0x00, 0x06, 0x00, 0x20, 0x22,
                                               0x21, 0x23})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kWriteSys,
                     std::vector<std::uint8_t>{0x07, 0x00, 0x00, 0x00, 0x00,
                                               0x00})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kWriteSys,
                     std::vector<std::uint8_t>{0x03, 0x00, 0x01, 0x00, 0x00,
                                               0x00})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kStartI2cTest,
                     std::vector<std::uint8_t>{0x00, 0x00, 0x03, 0x00, 0x00,
                                               0x00})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kFftZerofill,
                     std::vector<std::uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00,
                                               0x00})) {
    return false;
  }
  if (!SendWriteRegBatches(regs)) {
    AERROR << "ICL1122 WriteReg batches failed";
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kReadPdCalibration,
                     std::vector<std::uint8_t>{0x10, 0x00, 0x11, 0x00, 0x12,
                                               0x00, 0x13, 0x00})) {
    return false;
  }
  if (!WriteAndDrain(Icl1122HostCmd::kFinishCfg, {})) {
    return false;
  }
  // Let the radio settle before the async reader starts.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  FlushInput();
  AINFO << "ICL1122 host bring-up finished";
  return true;
}

}  // namespace protocol
}  // namespace autodriver
