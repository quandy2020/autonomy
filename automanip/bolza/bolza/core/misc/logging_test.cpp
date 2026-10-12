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

#include <gtest/gtest.h>

#include <array>
#include <fstream>
#include <sstream>
#include <string>

#include <bolza/core/misc/log.hpp>

namespace {
bool endsWith(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

TEST(testLogging, canPrintMessage) {
  bolza::log::Settings settings;

  settings.useLogFile = false;
  settings.useConsole = true;
  settings.consoleSeverity = bolza::log::SeverityLevel::DEBUG;

  bolza::log::Init(settings);

  BOLZA_DEBUG << "A debug severity message";
  BOLZA_INFO << "An informational severity message";
  BOLZA_WARN << "A warning severity message";
  BOLZA_ERROR << "An error severity message";
  BOLZA_INFO << settings;

  bolza::log::Reset();
}

TEST(testLogging, canWriteLogFile) {
  bolza::log::Settings settings;

  settings.useConsole = false;
  settings.useLogFile = true;
  settings.logFileSeverity = bolza::log::SeverityLevel::DEBUG;
  settings.logFileName = "log/testLogging.log";

  bolza::log::Init(settings);

  BOLZA_LOG(DEBUG) << "A debug severity message";
  BOLZA_LOG(INFO) << "An informational severity message";
  BOLZA_LOG(WARNING) << "A warning severity message";
  BOLZA_LOG(ERROR) << "An error severity message";

  bolza::log::Reset();

  std::ifstream file(settings.logFileName);
  std::array<std::string, 4> expected = {"[   DEBUG ] A debug severity message", "[    INFO ] An informational severity message",
                                         "[ WARNING ] A warning severity message", "[   ERROR ] An error severity message"};

  for (const auto& end : expected) {
    std::string line;
    std::getline(file, line);
    EXPECT_TRUE(endsWith(line, end));
  }
}

TEST(testLogging, writesCorrectMessageToConsole) {
  bolza::log::Settings settings;

  settings.useLogFile = false;
  settings.useConsole = true;
  settings.consoleSeverity = bolza::log::SeverityLevel::DEBUG;

  std::ostringstream console_stream;
  bolza::log::Init(settings, &console_stream);

  BOLZA_LOG(DEBUG) << "A debug severity message";
  BOLZA_LOG(INFO) << "An informational severity message";
  BOLZA_LOG(WARNING) << "A warning severity message";
  BOLZA_LOG(ERROR) << "An error severity message";

  const std::string expect =
      "[   DEBUG ] A debug severity message\n"
      "[    INFO ] An informational severity message\n"
      "[ WARNING ] A warning severity message\n"
      "[   ERROR ] An error severity message\n";

  EXPECT_EQ(console_stream.str(), expect);

  bolza::log::Reset();
}

TEST(testLogging, canFilterSeverity) {
  bolza::log::Settings settings;

  settings.useLogFile = false;
  settings.useConsole = true;
  settings.consoleSeverity = bolza::log::SeverityLevel::ERROR;

  std::ostringstream console_stream;
  bolza::log::Init(settings, &console_stream);

  BOLZA_LOG(DEBUG) << "NOT logged";
  BOLZA_LOG(INFO) << "NOT logged";
  BOLZA_LOG(WARNING) << "NOT logged";
  BOLZA_LOG(ERROR) << "Logged";

  EXPECT_EQ(console_stream.str(), "[   ERROR ] Logged\n");

  bolza::log::Reset();
}

TEST(testLogging, canLoadSettings) {
  const std::string settingsFileName = "bolza_test_log_settings.info";

  std::ofstream file(settingsFileName);
  file << R"(
; logging settings
log
{
  useConsole        1         ; enable console log
  consoleSeverity   INFO      ; console severity level
  useLogFile        1         ; enable file log
  logFileSeverity   WARNING   ; file severity level
  logFileName       bolza.log  ; log file name
}
)";
  file.close();

  bolza::log::Settings settings = bolza::log::LoadSettings(settingsFileName, "log");

  EXPECT_EQ(settings.useConsole, true);
  EXPECT_EQ(settings.consoleSeverity, bolza::log::SeverityLevel::INFO);
  EXPECT_EQ(settings.useLogFile, true);
  EXPECT_EQ(settings.logFileSeverity, bolza::log::SeverityLevel::WARNING);
  EXPECT_EQ(settings.logFileName, "bolza.log");
}
