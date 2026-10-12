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
#include <map>
#include <string>
#include <vector>


#include <bolza/core/misc/load_data.hpp>

#include "examples/legged_robot/common/types.hpp"

namespace bolza {
namespace legged_robot {

enum ModeNumber {  // {LF, RF, LH, RH}
  FLY = 0,
  RH = 1,
  LH = 2,
  LH_RH = 3,
  RF = 4,
  RF_RH = 5,
  RF_LH = 6,
  RF_LH_RH = 7,
  LF = 8,
  LF_RH = 9,
  LF_LH = 10,
  LF_LH_RH = 11,
  LF_RF = 12,
  LF_RF_RH = 13,
  LF_RF_LH = 14,
  STANCE = 15,
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t ModeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

  switch (modeNumber) {
    case 0:
      stanceLegs = contact_flag_t{false, false, false, false};
      break;  // 0:  0-leg-stance
    case 1:
      stanceLegs = contact_flag_t{false, false, false, true};
      break;  // 1:  RH
    case 2:
      stanceLegs = contact_flag_t{false, false, true, false};
      break;  // 2:  LH
    case 3:
      stanceLegs = contact_flag_t{false, false, true, true};
      break;  // 3:  RH, LH
    case 4:
      stanceLegs = contact_flag_t{false, true, false, false};
      break;  // 4:  RF
    case 5:
      stanceLegs = contact_flag_t{false, true, false, true};
      break;  // 5:  RF, RH
    case 6:
      stanceLegs = contact_flag_t{false, true, true, false};
      break;  // 6:  RF, LH
    case 7:
      stanceLegs = contact_flag_t{false, true, true, true};
      break;  // 7:  RF, LH, RH
    case 8:
      stanceLegs = contact_flag_t{true, false, false, false};
      break;  // 8:  LF,
    case 9:
      stanceLegs = contact_flag_t{true, false, false, true};
      break;  // 9:  LF, RH
    case 10:
      stanceLegs = contact_flag_t{true, false, true, false};
      break;  // 10: LF, LH
    case 11:
      stanceLegs = contact_flag_t{true, false, true, true};
      break;  // 11: LF, LH, RH
    case 12:
      stanceLegs = contact_flag_t{true, true, false, false};
      break;  // 12: LF, RF
    case 13:
      stanceLegs = contact_flag_t{true, true, false, true};
      break;  // 13: LF, RF, RH
    case 14:
      stanceLegs = contact_flag_t{true, true, true, false};
      break;  // 14: LF, RF, LH
    case 15:
      stanceLegs = contact_flag_t{true, true, true, true};
      break;  // 15: 4-leg-stance
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t StanceLeg2ModeNumber(const contact_flag_t& stanceLegs) {
  return static_cast<size_t>(stanceLegs[3]) + 2 * static_cast<size_t>(stanceLegs[2]) + 4 * static_cast<size_t>(stanceLegs[1]) +
         8 * static_cast<size_t>(stanceLegs[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string ModeNumber2String(const size_t& modeNumber) {
  // build the map from mode number to name
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[RH] = "RH";
  modeToName[LH] = "LH";
  modeToName[LH_RH] = "LH_RH";
  modeToName[RF] = "RF";
  modeToName[RF_RH] = "RF_RH";
  modeToName[RF_LH] = "RF_LH";
  modeToName[RF_LH_RH] = "RF_LH_RH";
  modeToName[LF] = "LF";
  modeToName[LF_RH] = "LF_RH";
  modeToName[LF_LH] = "LF_LH";
  modeToName[LF_LH_RH] = "LF_LH_RH";
  modeToName[LF_RF] = "LF_RF";
  modeToName[LF_RF_RH] = "LF_RF_RH";
  modeToName[LF_RF_LH] = "LF_RF_LH";
  modeToName[STANCE] = "STANCE";

  return modeToName[modeNumber];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t String2ModeNumber(const std::string& modeString) {
  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["RH"] = RH;
  nameToMode["LH"] = LH;
  nameToMode["LH_RH"] = LH_RH;
  nameToMode["RF"] = RF;
  nameToMode["RF_RH"] = RF_RH;
  nameToMode["RF_LH"] = RF_LH;
  nameToMode["RF_LH_RH"] = RF_LH_RH;
  nameToMode["LF"] = LF;
  nameToMode["LF_RH"] = LF_RH;
  nameToMode["LF_LH"] = LF_LH;
  nameToMode["LF_LH_RH"] = LF_LH_RH;
  nameToMode["LF_RF"] = LF_RF;
  nameToMode["LF_RF_RH"] = LF_RF_RH;
  nameToMode["LF_RF_LH"] = LF_RF_LH;
  nameToMode["STANCE"] = STANCE;

  return nameToMode[modeString];
}

}  // namespace legged_robot
}  // end of namespace bolza
