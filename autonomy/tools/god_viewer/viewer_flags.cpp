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

#include "autonomy/tools/god_viewer/viewer_flags.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer {

DEFINE_string(foxglove_config_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Autonomy installation to allow "
              "including files from there.");
DEFINE_string(foxglove_config_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

}   // namespace god_viewer
}   // namespace tools
}   // namespace autonomy