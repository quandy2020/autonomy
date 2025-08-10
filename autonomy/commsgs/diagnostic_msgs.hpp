/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once 

#include <vector>
#include <string>

#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace diagnostic_msgs {

struct KeyValue
{
    // What to label this value when viewing.
    std::string key;
    
    // A value to track over time.
    std::string value;
};

// This message holds the status of an individual component of the robot.
struct DiagnosticStatus
{
    // // Possible levels of operations.
    // byte OK=0
    // byte WARN=1
    // byte ERROR=2
    // byte STALE=3
    // Level of operation enumerated above.
    uint8 level;

    // A description of the test/component reporting.
    std::string name;

    // A description of the status.
    std::string message;

    // A hardware unique std::string.
    std::string hardware_id;

    // An array of values associated with the status.
    std::vector<KeyValue> values;
};

// This message is used to send diagnostic information about the state of the robot.
struct DiagnosticArray
{
    // for timestamp
    std_msgs::Header header;

    // an array of components being reported on
    std::vector<DiagnosticStatus> status;
};

}  // namespace diagnostic_msgs
}  // namespace commsgs
}  // namespace autonomy