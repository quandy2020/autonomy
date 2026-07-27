/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/transformation_manager.hpp"

/** Export from Autoviz transformer plugin shared libraries. */
#define AUTOVIZ_TRANSFORMER_PLUGIN_EXPORT extern "C" void autoviz_register_transformers

namespace autoviz {
namespace common {

inline void RegisterTransformerPlugin(
    TransformationManager* manager, const char* class_id,
    FrameTransformerCreator creator) {
  if (manager != nullptr) {
    manager->registerTransformer(class_id, std::move(creator));
  }
}

}  // namespace common
}  // namespace autoviz
