/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Helpers for Protobuf JSON util API differences across 3.19 / 22+ / 35+.
 *****************************************************************************/

/**
 * @file protobuf_json_compat.hpp
 * @brief Compatibility shims for Protobuf JSON util across major versions.
 *
 * Field names and status message APIs changed between Protobuf 3.19, 22+,
 * and 35+. These helpers keep Autoviz call sites version-agnostic.
 *
 * @see google::protobuf::util::JsonPrintOptions
 */

#pragma once

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/util/json_util.h>

#include <QString>
#include <string>

namespace autoviz {

/**
 * @brief Enables “always print primitive / no-presence fields” on @p options.
 *
 * Uses @c always_print_fields_with_no_presence on Protobuf ≥ 4.26 and
 * @c always_print_primitive_fields on older versions.
 *
 * @param options Print options to mutate; no-op if @c nullptr.
 */
inline void SetAlwaysPrintPrimitiveFields(
    google::protobuf::util::JsonPrintOptions* options) {
  if (options == nullptr) {
    return;
  }
#if GOOGLE_PROTOBUF_VERSION >= 4026000
  options->always_print_fields_with_no_presence = true;
#else
  options->always_print_primitive_fields = true;
#endif
}

/**
 * @brief Extracts a status message as @c QString (version-agnostic).
 *
 * @tparam StatusT Status type with @c message() returning string-like bytes.
 * @param status Status object (e.g. @c absl::Status).
 * @return UTF-8 Qt string of the message.
 */
template <typename StatusT>
inline QString StatusMessageToQString(const StatusT& status) {
  const auto message = status.message();
  return QString::fromUtf8(message.data(), static_cast<int>(message.size()));
}

/**
 * @brief Extracts a status message as @c std::string (version-agnostic).
 *
 * @tparam StatusT Status type with @c message() returning string-like bytes.
 * @param status Status object.
 * @return UTF-8 std string of the message.
 */
template <typename StatusT>
inline std::string StatusMessageToStdString(const StatusT& status) {
  const auto message = status.message();
  return std::string(message.data(), message.size());
}

}  // namespace autoviz
