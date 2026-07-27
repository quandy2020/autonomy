/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/properties/property.hpp"

namespace autoviz {
namespace common {

std::unique_ptr<Property> CreatePropertyFromSpec(const DisplayPropertySpec& spec,
                                                 const std::string& value);

std::unique_ptr<Property> BuildPropertyTreeFromSpecs(
    const std::vector<DisplayPropertySpec>& specs,
    const DisplayPropertyMap& values);

void SyncPropertyTreeFromMap(Property* root, const DisplayPropertyMap& values);
void SyncPropertyTreeToMap(const Property& root, DisplayPropertyMap* values);

}  // namespace common
}  // namespace autoviz
