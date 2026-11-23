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
 * @file host_cfg.hpp
 * @brief JoySpace / 24GMmwave host command packets + EVB1122 bring-up.
 *
 * Wire format (CmdPacket):
 *   FD FC FB FA | len_u16 LE | cmd_u16 LE | payload[len-2] | 04 03 02 01
 * where `len` includes the 2-byte cmd id.
 */

#ifndef AUTODRIVER_RADAR_ICL1122_HOST_CFG_HPP_
#define AUTODRIVER_RADAR_ICL1122_HOST_CFG_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "autodriver/common/stream.hpp"

namespace autodriver {
namespace protocol {

/** @brief JoySpace CmdType values used by Mmwave::StartSensor. */
enum class Icl1122HostCmd : std::uint16_t {
  kReadVer = 0x0000,
  kWriteReg = 0x0001,
  kWriteSys = 0x0012,
  kCascadingMode = 0x0014,
  kFftZerofill = 0x0015,
  kStartI2cTest = 0x0020,
  kReadPdCalibration = 0x0027,
  kGetAppRun = 0x0060,
  kGetWhichDemo = 0x0066,
  kFinishCfg = 0x00FE,
  kStartCfg = 0x00FF,
};

/**
 * @brief Build one host command packet (FD…01).
 * @param[in] cmd Command id.
 * @param[in] payload Bytes after cmd (may be empty).
 */
std::vector<std::uint8_t> BuildIcl1122HostPacket(
    Icl1122HostCmd cmd, const std::vector<std::uint8_t>& payload);

/**
 * @brief Parse `addr=value` hex lines from a JoySpace RegisterConfig file.
 * @param[in] path Absolute or relative path to the register file.
 * @param[out] out_pairs Cleared then filled with (addr, value) LE pairs.
 * @return true when the file opened and at least one pair was parsed.
 */
bool LoadIcl1122RegisterConfig(
    const std::string& path,
    std::vector<std::pair<std::uint16_t, std::uint16_t>>* out_pairs);

/**
 * @class autodriver::protocol::Icl1122HostConfigurator
 * @brief Sends the JoySpace StartSensor command sequence over a Stream.
 */
class Icl1122HostConfigurator {
 public:
  /**
   * @brief Construct with transport and RegisterConfig path.
   * @param[in] stream Connected Stream (not owned).
   * @param[in] register_config_path JoySpace register file.
   * @param[in] response_timeout_ms Drain/wait after each cmd (vendor uses 1500).
   */
  Icl1122HostConfigurator(common::Stream* stream,
                          std::string register_config_path,
                          int response_timeout_ms = 200);

  /**
   * @brief Run the Mmwave::StartSensor bring-up sequence.
   * @return true when register writes completed; false on I/O or parse failure.
   */
  bool Run();

 private:
  bool WritePacket(Icl1122HostCmd cmd,
                   const std::vector<std::uint8_t>& payload);
  bool WriteAndDrain(Icl1122HostCmd cmd,
                     const std::vector<std::uint8_t>& payload);
  bool Drain(int timeout_ms);
  bool FlushInput();
  bool SendWriteRegBatches(
      const std::vector<std::pair<std::uint16_t, std::uint16_t>>& pairs);

  common::Stream* stream_{nullptr};
  std::string register_config_path_;
  int response_timeout_ms_{200};
};

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_ICL1122_HOST_CFG_HPP_
