/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Helpers for Protobuf JSON util API differences across 3.19 / 22+ / 35+.
 *****************************************************************************/

#pragma once

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/util/json_util.h>

#include <QString>
#include <string>

namespace autoviz {

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

template <typename StatusT>
inline QString StatusMessageToQString(const StatusT& status) {
  const auto message = status.message();
  return QString::fromUtf8(message.data(), static_cast<int>(message.size()));
}

template <typename StatusT>
inline std::string StatusMessageToStdString(const StatusT& status) {
  const auto message = status.message();
  return std::string(message.data(), message.size());
}

}  // namespace autoviz
