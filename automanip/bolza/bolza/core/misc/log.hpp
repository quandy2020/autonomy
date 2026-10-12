/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#pragma once

/**
 * @file log.hpp
 * @brief Lightweight severity logger (C++17 iostreams, no Boost.Log).
 */

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

namespace bolza {
namespace log {

enum class SeverityLevel : size_t { DEBUG, INFO, WARNING, ERROR };

const std::string& ToString(SeverityLevel lvl);
SeverityLevel FromString(const std::string& severity);

template <typename CharT, typename TraitsT>
inline std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& strm,
                                                      SeverityLevel lvl) {
  strm << ToString(lvl);
  return strm;
}

struct Settings {
  bool useConsole = true;
  SeverityLevel consoleSeverity = SeverityLevel::DEBUG;
  bool useLogFile = false;
  SeverityLevel logFileSeverity = SeverityLevel::INFO;
  std::string logFileName = "bolza.log";
};

Settings LoadSettings(const std::string& fileName, const std::string& fieldName);
std::ostream& operator<<(std::ostream& stream, const Settings& settings);

void Init(const Settings& settings, std::ostream* console_stream = &std::clog);
void Reset();

class Logger {
 public:
  class Record {
   public:
    Record(SeverityLevel level, Logger* owner) : level_(level), owner_(owner) {}
    Record(const Record&) = delete;
    Record& operator=(const Record&) = delete;
    Record(Record&& other) noexcept
        : level_(other.level_), owner_(other.owner_), stream_(std::move(other.stream_)) {
      other.owner_ = nullptr;
    }
    Record& operator=(Record&&) = delete;
    ~Record();

    template <typename T>
    Record& operator<<(const T& value) {
      stream_ << value;
      return *this;
    }

   private:
    SeverityLevel level_;
    Logger* owner_;
    std::ostringstream stream_;
  };

  Record Stream(SeverityLevel level) { return Record(level, this); }
  void Write(SeverityLevel level, const std::string& message);

 private:
  friend void Init(const Settings&, std::ostream*);
  friend void Reset();

  std::mutex mutex_;
  Settings settings_;
  std::ostream* console_ = nullptr;
  std::unique_ptr<std::ofstream> file_;
};

Logger& GetLogger();

#define BOLZA_LOG(LVL) ::bolza::log::GetLogger().Stream(::bolza::log::SeverityLevel::LVL)
#define BOLZA_DEBUG BOLZA_LOG(DEBUG)
#define BOLZA_INFO BOLZA_LOG(INFO)
#define BOLZA_WARN BOLZA_LOG(WARNING)
#define BOLZA_ERROR BOLZA_LOG(ERROR)

// Legacy aliases
#define OCS2_LOG BOLZA_LOG
#define OCS2_DEBUG BOLZA_DEBUG
#define OCS2_INFO BOLZA_INFO
#define OCS2_WARN BOLZA_WARN
#define OCS2_ERROR BOLZA_ERROR

}  // namespace log
}  // namespace bolza
