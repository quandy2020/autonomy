/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include "autonomy/task/teleop/teleop_mppi_assist.hpp"

namespace autonomy::task::teleop {

constexpr char kDefaultTeleopAssistConfigRelPath[] = "task/teleop_assist.lua";

/** Loads teleop assist options; on missing file returns `enabled=false`. */
TeleopMppiAssist::Options LoadTeleopAssistOptions(
    const std::string& config_directory,
    const std::string& relative_path = kDefaultTeleopAssistConfigRelPath);

}  // namespace autonomy::task::teleop
