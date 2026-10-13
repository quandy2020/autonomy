/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <string>

namespace autolink {
namespace logger {

/**
 * Publish a foxglove.Log JSON payload on /rosout so Autoviz Log panel (and
 * other subscribers) can see logs from every process that uses AsyncLogger.
 * Safe to call from the async log thread; drops on re-entrancy / init failure.
 */
void BroadcastLogToRosout(int severity_level, const std::string& module_name,
                          const std::string& message, int64_t timestamp_sec);

}  // namespace logger
}  // namespace autolink
