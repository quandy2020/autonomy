/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file property_factory.hpp
 * @brief Build and sync @ref Property trees from @ref DisplayPropertySpec lists.
 *
 * Converts display/tool property schemas and string maps into hierarchical
 * @ref Property trees for the Displays panel, and syncs values both ways.
 *
 * @see Property
 * @see PropertyTreeBuilder
 * @see DisplayPropertySpec
 */

#pragma once

#include <memory>
#include <vector>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/properties/property.hpp"

namespace autoviz {
namespace common {

/**
 * @brief Creates a single leaf @ref Property from a spec and current value.
 *
 * Chooses @ref StringProperty, @ref BoolProperty, @ref ColorProperty,
 * @ref EnumProperty, etc. based on @p spec.kind / options.
 *
 * @param spec Property schema entry.
 * @param value Current string value (often from @ref DisplayPropertyMap).
 * @return New property node (never @c nullptr for a valid spec).
 */
std::unique_ptr<Property> CreatePropertyFromSpec(const DisplayPropertySpec& spec,
                                                 const std::string& value);

/**
 * @brief Builds a full property tree from specs and a value map.
 *
 * Honors @c parent_key nesting and vector expansion from the specs.
 *
 * @param specs Ordered property schema.
 * @param values Current key/value map.
 * @return Root group property owning the tree.
 */
std::unique_ptr<Property> BuildPropertyTreeFromSpecs(
    const std::vector<DisplayPropertySpec>& specs,
    const DisplayPropertyMap& values);

/**
 * @brief Pushes map values into an existing tree (by leaf name).
 *
 * @param root Tree root (non-null).
 * @param values Source key/value map.
 */
void SyncPropertyTreeFromMap(Property* root, const DisplayPropertyMap& values);

/**
 * @brief Reads leaf values from a tree into a property map.
 *
 * @param root Tree root.
 * @param[out] values Destination map (non-null); keys updated/inserted.
 */
void SyncPropertyTreeToMap(const Property& root, DisplayPropertyMap* values);

}  // namespace common
}  // namespace autoviz
