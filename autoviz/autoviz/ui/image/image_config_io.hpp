/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_config_io.hpp
 * @brief Convert Image panel runtime config ↔ session persist config.
 *
 * Used by VisualizationFrame / FrameSession when saving and restoring
 * @ref ImagePanelConfig into @ref common::SessionConfig.
 *
 * @see ImagePanelConfig
 * @see common::ImagePanelPersistConfig
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/image/image_types.hpp"

namespace autoviz {
namespace image {

/**
 * @brief Serializes an @ref ImagePanelConfig into the session persist form.
 *
 * @param object_name Qt object name of the panel (dock identity key).
 * @param config Runtime panel configuration.
 * @return Persist struct suitable for @ref common::SessionConfig.
 * @see FromPersistConfig()
 */
common::ImagePanelPersistConfig ToPersistConfig(const QString& object_name,
                                               const ImagePanelConfig& config);

/**
 * @brief Restores an @ref ImagePanelConfig from session persist data.
 *
 * @param persist Saved panel config from session / layout file.
 * @return Runtime config applied via @ref ImagePanel::setConfig().
 * @see ToPersistConfig()
 */
ImagePanelConfig FromPersistConfig(const common::ImagePanelPersistConfig& persist);

}  // namespace image
}  // namespace autoviz
