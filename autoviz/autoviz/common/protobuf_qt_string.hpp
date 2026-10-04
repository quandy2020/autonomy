/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file protobuf_qt_string.hpp
 * @brief Convert protobuf string / string_view fields to @c QString.
 *
 * Abstracts @c absl::string_view vs @c std::string_view differences across
 * Protobuf versions when reading @c name() and similar accessors.
 */

#pragma once

#include <QString>

namespace autoviz {

/**
 * @brief Builds a UTF-8 @c QString from any string-like with @c data()/size().
 *
 * @tparam StringLike Type exposing @c data() and @c size() (e.g. string_view).
 * @param value Source bytes.
 * @return Qt string (empty if @p value is empty).
 */
template <typename StringLike>
inline QString ProtobufToQString(const StringLike& value) {
  return QString::fromUtf8(value.data(), static_cast<int>(value.size()));
}

}  // namespace autoviz
