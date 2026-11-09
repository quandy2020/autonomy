/* Copyright 2026 The Navigation Authors (navigation) */
#pragma once

#include <string>

#include "navigation/common/status.hpp"

namespace navigation::common {

/** @brief Reject missing files, invalid hashes and mismatched model bytes. */
Status ValidateModelIntegrity(const std::string& path,
                              const std::string& sha256);

}  // namespace navigation::common
