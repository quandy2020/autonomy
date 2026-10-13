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

#ifndef AUTOMSGS_MSGS_INSTALLATION_DIRECTORIES_HH_
#define AUTOMSGS_MSGS_INSTALLATION_DIRECTORIES_HH_

#include <string>

#include <automsgs/msgs/config.hh>
#include <automsgs/msgs/Export.hh>

namespace automsgs {
namespace msgs {

/// \brief Returns the install prefix of the library (CMAKE_INSTALL_PREFIX at
/// build time). Can be overridden at runtime by the environment variable
/// AUTOMSGS_INSTALL_PREFIX.
AUTOMSGS_MSGS_VISIBLE std::string getInstallPrefix();

}  // namespace msgs
}  // namespace automsgs

#endif  // AUTOMSGS_MSGS_INSTALLATION_DIRECTORIES_HH_
