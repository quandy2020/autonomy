/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file message_type_utils.hpp
 * @brief Normalize and compare Automsgs message type descriptor strings.
 *
 * Topology / session configs may still carry legacy @c automsgs.msgs.* forms;
 * UI and writers use these helpers before schema checks and channel matching.
 *
 * @see NormalizeMessageType
 * @see MessageTypesCompatible
 * @see integration::ChannelWriterRegistry
 */

#pragma once

#include <string>

namespace autoviz {
namespace commsgs {

/**
 * @brief Maps legacy Automsgs descriptors to the canonical form.
 *
 * Example: legacy package-style names are rewritten to the current
 * @c automsgs.msgs.* descriptor spelling used by Autolink / protobuf.
 *
 * @param message_type Raw type string from topology, session, or UI.
 * @return Normalized descriptor; may equal @p message_type if already canonical
 *         or unrecognized.
 */
std::string NormalizeMessageType(const std::string& message_type);

/**
 * @brief Returns whether two descriptors refer to the same logical message.
 *
 * Compares after @ref NormalizeMessageType so legacy and current spellings
 * match.
 *
 * @param left First type descriptor.
 * @param right Second type descriptor.
 * @return @c true if both normalize to the same logical message type.
 */
bool MessageTypesCompatible(const std::string& left, const std::string& right);

}  // namespace commsgs
}  // namespace autoviz
