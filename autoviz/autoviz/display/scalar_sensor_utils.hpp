/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>

namespace autoviz {
namespace display {

QColor colorFromScalar(double value, double min_value, double max_value);

}  // namespace display
}  // namespace autoviz
