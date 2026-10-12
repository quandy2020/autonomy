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

#include "examples/legged_robot/gait/mode_sequence_template.hpp"

#include <bolza/core/misc/display.hpp>
#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& modeSequenceTemplate) {
  stream << "Template switching times: {" << ToDelimitedString(modeSequenceTemplate.switchingTimes) << "}\n";
  stream << "Template mode sequence:   {" << ToDelimitedString(modeSequenceTemplate.modeSequence) << "}\n";
  return stream;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate LoadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose) {
  std::vector<scalar_t> switchingTimes;
  loadData::LoadStdVector(filename, topicName + ".switchingTimes", switchingTimes, verbose);

  std::vector<std::string> modeSequenceString;
  loadData::LoadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

  if (switchingTimes.empty() || modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(String2ModeNumber(modeName));
  }

  return {switchingTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Gait ToGait(const ModeSequenceTemplate& modeSequenceTemplate) {
  const auto startTime = modeSequenceTemplate.switchingTimes.front();
  const auto endTime = modeSequenceTemplate.switchingTimes.back();
  Gait gait;
  gait.duration = endTime - startTime;
  // Events: from time -> phase
  gait.eventPhases.reserve(modeSequenceTemplate.switchingTimes.size());
  std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                [&](scalar_t eventTime) { gait.eventPhases.push_back((eventTime - startTime) / gait.duration); });
  // Modes:
  gait.modeSequence = modeSequenceTemplate.modeSequence;
  assert(IsValidGait(gait));
  return gait;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule LoadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose) {
  std::vector<scalar_t> eventTimes;
  loadData::LoadStdVector(filename, topicName + ".eventTimes", eventTimes, verbose);

  std::vector<std::string> modeSequenceString;
  loadData::LoadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

  if (modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSchedule] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(String2ModeNumber(modeName));
  }

  return {eventTimes, modeSequence};
}

}  // namespace legged_robot
}  // namespace bolza
