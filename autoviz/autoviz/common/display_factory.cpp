/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/display_factory.hpp"

#include "autoviz/common/display_registry.hpp"

namespace autoviz {
namespace common {

std::vector<std::string> DisplayFactory::supportedTypes() {
  return DisplayRegistry::instance().supportedTypes();
}

DisplayConfig DisplayFactory::defaultForType(const std::string& type) {
  return DisplayRegistry::instance().defaultForType(type);
}

std::unique_ptr<display::Display> DisplayFactory::create(
    const DisplayConfig& config) {
  return DisplayRegistry::instance().create(config);
}

}  // namespace common
}  // namespace autoviz
