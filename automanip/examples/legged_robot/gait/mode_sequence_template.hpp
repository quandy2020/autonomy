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

#pragma once

#include <iostream>
#include <vector>

#include <bolza/core/reference/mode_schedule.hpp>

#include "examples/legged_robot/gait/gait.hpp"
#include "examples/legged_robot/gait/motion_phase_definition.hpp"

namespace bolza {
namespace legged_robot {

/**
 * ModeSequenceTemplate describes a periodic sequence of modes. It is defined by
 * - switching times (size N+1), where the first time is 0, and the last time denotes the period of the cycle
 * - modeSequence (size N), indicating the mode between the switching times.
 */
struct ModeSequenceTemplate {
  /**
   * Constructor for a ModeSequenceTemplate. The number of modes must be greater than Zero (N > 0)
   * @param [in] switchingTimesInput : switching times of size N + 1
   * @param [in] modeSequenceInput : mode sequence of size N
   */
  ModeSequenceTemplate(std::vector<scalar_t> switchingTimesInput, std::vector<size_t> modeSequenceInput)
      : SwitchingTimes(std::move(switchingTimesInput)), modeSequence(std::move(modeSequenceInput)) {
    assert(!modeSequence.empty());
    assert(switchingTimes.size() == modeSequence.size() + 1);
  }

  /**
   * Defined as [t_0=0, t_1, .., t_n, t_(n+1)=T], where T is the overall duration
   * of the template logic. t_1 to t_n are the event moments.
   */
  std::vector<scalar_t> switchingTimes;

  /**
   * Defined as [sys_0, sys_n], are the switching systems IDs. Here sys_i is
   * active in period [t_i, t_(i+1)]
   */
  std::vector<size_t> modeSequence;
};

/** Swap two modesequence templates */
inline void swap(ModeSequenceTemplate& lh, ModeSequenceTemplate& rh) {
  lh.switchingTimes.swap(rh.switchingTimes);
  lh.modeSequence.swap(rh.modeSequence);
}

/** Print the modesequence template */
std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& modeSequenceTemplate);

/** Converts a mode sequence template to a gait */
Gait ToGait(const ModeSequenceTemplate& modeSequenceTemplate);

/**
 * Load a modesequence template from file.  The template needs to be declared as:
 *
 * topicName
 * {
 *   modeSequence
 *   {
 *     [0]     mode0
 *     [1]     mode1
 *   }
 *   switchingTimes
 *   {
 *     [0]     0.0
 *     [1]     t1
 *     [2]     T
 *   }
 * }
 */
ModeSequenceTemplate LoadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose = true);

/**
 * Load a mode schedule template from file.  The schedule needs to be declared as:
 *
 * topicName
 * {
 *   modeSequence
 *   {
 *     [0]     mode0
 *     [1]     mode1
 *     [2]     mode2
 *   }
 *   eventTimes
 *   {
 *     [0]     t0
 *     [1]     t1
 *   }
 * }
 */
ModeSchedule LoadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose);

}  // namespace legged_robot
}  // namespace bolza
