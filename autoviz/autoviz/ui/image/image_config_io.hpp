/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/image/image_types.hpp"

namespace autoviz {
namespace image {

common::ImagePanelPersistConfig ToPersistConfig(const QString& object_name,
                                               const ImagePanelConfig& config);
ImagePanelConfig FromPersistConfig(const common::ImagePanelPersistConfig& persist);

}  // namespace image
}  // namespace autoviz
