/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file transformer_plugin.hpp
 * @brief Macro and helper for Autoviz FrameTransformer plugin shared libraries.
 *
 * @see TransformationManager
 * @see FrameTransformer
 * @see AUTOVIZ_TRANSFORMER_PLUGIN_EXPORT
 */

#pragma once

#include "autoviz/common/transformation_manager.hpp"

/**
 * @brief Declares the C export for transformer plugins.
 *
 * Expands to @c extern "C" void autoviz_register_transformers.
 */
#define AUTOVIZ_TRANSFORMER_PLUGIN_EXPORT extern "C" void autoviz_register_transformers

namespace autoviz {
namespace common {

/**
 * @brief Registers one FrameTransformer creator into @p manager (null-safe).
 *
 * @param manager Target @ref TransformationManager (no-op if @c nullptr).
 * @param class_id Plugin class id (e.g. @c "autoviz/AutolinkTf").
 * @param creator Factory producing a @ref FrameTransformer.
 */
inline void RegisterTransformerPlugin(
    TransformationManager* manager, const char* class_id,
    FrameTransformerCreator creator) {
  if (manager != nullptr) {
    manager->registerTransformer(class_id, std::move(creator));
  }
}

}  // namespace common
}  // namespace autoviz
