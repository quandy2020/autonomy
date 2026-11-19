// SPDX-License-Identifier: Apache-2.0
/** @file configuration_loader.hpp
 * @brief Load the complete coverage runtime defaults from JSON. */
#pragma once
#include "autonomy/task/navigation/coverage/configuration.hpp"
namespace autonomy::task::coverage {
/** @brief Read a runtime configuration; throw on missing files or invalid
 * values. */
Configuration LoadConfiguration(const std::string& filename);
}  // namespace autonomy::task::coverage
