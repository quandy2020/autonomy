/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_config_io.hpp
 * @brief Convert Plot panel runtime config ↔ session persist config.
 *
 * Used when saving / restoring @ref PlotPanelConfig into
 * @ref common::SessionConfig.
 *
 * @see PlotPanelConfig
 * @see common::PlotPanelPersistConfig
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

/**
 * @brief Serializes a plot panel config into the session persist form.
 *
 * @param object_name Qt object name of the panel (dock identity key).
 * @param config Runtime panel configuration.
 * @return Persist struct for @ref common::SessionConfig.
 * @see FromPersistConfig()
 */
common::PlotPanelPersistConfig ToPersistConfig(const QString& object_name,
                                               const PlotPanelConfig& config);

/**
 * @brief Restores a plot panel config from session persist data.
 *
 * @param persist Saved panel config.
 * @return Runtime config for @ref PlotPanel::setConfig().
 * @see ToPersistConfig()
 */
PlotPanelConfig FromPersistConfig(const common::PlotPanelPersistConfig& persist);

}  // namespace plot
}  // namespace autoviz
