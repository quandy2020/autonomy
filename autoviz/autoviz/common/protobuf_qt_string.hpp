/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

namespace autoviz {

/** Convert protobuf name()/string_view (absl or std) to QString. */
template <typename StringLike>
inline QString ProtobufToQString(const StringLike& value) {
  return QString::fromUtf8(value.data(), static_cast<int>(value.size()));
}

}  // namespace autoviz
