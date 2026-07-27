/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

namespace autoviz {
namespace commsgs {

/** Map legacy autonomy.commsgs.proto.* descriptors to automsgs.msgs.*. */
std::string NormalizeMessageType(const std::string& message_type);

/** True if both descriptors refer to the same logical message. */
bool MessageTypesCompatible(const std::string& left, const std::string& right);

}  // namespace commsgs
}  // namespace autoviz
