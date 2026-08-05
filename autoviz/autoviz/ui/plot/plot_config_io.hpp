/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

common::PlotPanelPersistConfig ToPersistConfig(const QString& object_name,
                                               const PlotPanelConfig& config);
PlotPanelConfig FromPersistConfig(const common::PlotPanelPersistConfig& persist);

}  // namespace plot
}  // namespace autoviz
