/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QObject>
#include <QVariant>

namespace autoviz::widget::internal {

inline constexpr char kValidationErrorProperty[] = "autovizValidationError";

inline bool HasValidationError(const QObject* object) noexcept {
  return object != nullptr &&
         object->property(kValidationErrorProperty).toBool();
}

}  // namespace autoviz::widget::internal
