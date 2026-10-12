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

#include <bolza/core/misc/log.hpp>

#include <iomanip>
#include <unordered_map>

#include <bolza/core/misc/load_data.hpp>
#include <bolza/core/misc/property_tree.hpp>

namespace bolza {
namespace log {

Logger::Record::~Record() {
  if (owner_ != nullptr) {
    owner_->Write(level_, stream_.str());
  }
}

void Logger::Write(SeverityLevel level, const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream line;
  line << "[ " << std::setw(7) << std::setfill(' ') << ToString(level) << " ] " << message << '\n';
  const std::string text = line.str();

  if (settings_.useConsole && console_ != nullptr && level >= settings_.consoleSeverity) {
    (*console_) << text;
    console_->flush();
  }
  if (settings_.useLogFile && file_ && file_->good() && level >= settings_.logFileSeverity) {
    (*file_) << text;
    file_->flush();
  }
}

const std::string& ToString(const SeverityLevel lvl) {
  static const std::unordered_map<SeverityLevel, std::string> severityMap = {
      {SeverityLevel::DEBUG, "DEBUG"},
      {SeverityLevel::INFO, "INFO"},
      {SeverityLevel::WARNING, "WARNING"},
      {SeverityLevel::ERROR, "ERROR"}};
  return severityMap.at(lvl);
}

SeverityLevel FromString(const std::string& severity) {
  static const std::unordered_map<std::string, SeverityLevel> severityMap = {
      {"DEBUG", SeverityLevel::DEBUG},
      {"INFO", SeverityLevel::INFO},
      {"WARNING", SeverityLevel::WARNING},
      {"ERROR", SeverityLevel::ERROR}};
  return severityMap.at(severity);
}

Settings LoadSettings(const std::string& fileName, const std::string& fieldName) {
  Settings settings;
  property_tree::ptree pt;
  property_tree::read_info(fileName, pt);

  loadData::LoadPtreeValue(pt, settings.useConsole, fieldName + ".useConsole", false);

  std::string consoleSeverity = ToString(settings.consoleSeverity);
  loadData::LoadPtreeValue(pt, consoleSeverity, fieldName + ".consoleSeverity", false);
  settings.consoleSeverity = FromString(consoleSeverity);

  loadData::LoadPtreeValue(pt, settings.useLogFile, fieldName + ".useLogFile", false);

  std::string logFileSeverity = ToString(settings.logFileSeverity);
  loadData::LoadPtreeValue(pt, logFileSeverity, fieldName + ".logFileSeverity", false);
  settings.logFileSeverity = FromString(logFileSeverity);

  loadData::LoadPtreeValue(pt, settings.logFileName, fieldName + ".logFileName", false);
  return settings;
}

std::ostream& operator<<(std::ostream& stream, const Settings& settings) {
  stream << "\n #### Log Settings:\n";
  stream << " #### =============================================================================\n";
  loadData::PrintValue(stream, settings.useConsole, "useConsole");
  loadData::PrintValue(stream, settings.consoleSeverity, "consoleSeverity");
  loadData::PrintValue(stream, settings.useLogFile, "useLogFile");
  loadData::PrintValue(stream, settings.logFileSeverity, "logFileSeverity");
  loadData::PrintValue(stream, settings.logFileName, "logFileName");
  stream << " #### =============================================================================\n";
  return stream;
}

void Init(const Settings& settings, std::ostream* console_stream) {
  auto& logger = GetLogger();
  std::lock_guard<std::mutex> lock(logger.mutex_);
  logger.settings_ = settings;
  logger.console_ = console_stream;
  logger.file_.reset();
  if (settings.useLogFile) {
    logger.file_ = std::make_unique<std::ofstream>(settings.logFileName, std::ios::out | std::ios::app);
  }
}

void Reset() {
  auto& logger = GetLogger();
  std::lock_guard<std::mutex> lock(logger.mutex_);
  logger.console_ = nullptr;
  logger.file_.reset();
  logger.settings_ = Settings{};
}

Logger& GetLogger() {
  static Logger logger;
  return logger;
}

}  // namespace log
}  // namespace bolza
