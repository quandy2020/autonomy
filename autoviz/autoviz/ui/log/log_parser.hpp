/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include "autoviz/ui/log/log_types.hpp"

namespace autoviz {
namespace log_panel {

bool isLogMessageType(const std::string& message_type);
LogEntry logEntryFromPayload(const std::string& message_type,
                             const std::string& payload,
                             const QString& source);

}  // namespace log_panel
}  // namespace autoviz
