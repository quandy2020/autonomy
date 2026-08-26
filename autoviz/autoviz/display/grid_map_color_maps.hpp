/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>

namespace autoviz {
namespace display {

/** Sample a named colormap at normalized intensity t ∈ [0, 1]. */
QColor SampleGridMapColorMap(const std::string& name, float t);

std::vector<std::string> GridMapColorMapNames();

}  // namespace display
}  // namespace autoviz
