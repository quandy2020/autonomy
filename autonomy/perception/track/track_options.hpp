/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file
 * @brief Lua / default loaders for TrackOptions protobuf.
 */

#pragma once

#include <string>

#include "autonomy/perception/proto/track_options.pb.h"

namespace autonomy::perception::track {

/**
 * @brief Builds default tracking options for autosim bring-up.
 */
proto::TrackOptions DefaultOptions();

/**
 * @brief Fills missing / non-positive fields from defaults (in place).
 * @param options Options to sanitize; must be non-null.
 */
void ApplyDefaults(proto::TrackOptions* options);

/**
 * @brief Loads tracking options from a Lua file under |config_directory|.
 * @param config_directory Autonomy config root.
 * @param relative_path Path relative to the config root.
 * @return Parsed options, or defaults when loading fails (warns on error).
 */
proto::TrackOptions LoadOptions(
    const std::string& config_directory,
    const std::string& relative_path = "perception/track_yopo.lua");

}  // namespace autonomy::perception::track
