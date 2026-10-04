/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * Fluent builder for rviz-style grouped Display property trees.
 *****************************************************************************/

/**
 * @file property_tree_builder.hpp
 * @brief Fluent builder for RViz-style grouped @ref Property trees.
 *
 * Display implementations use this to assemble nested groups and typed leaves
 * while reading current values through a @ref Getter callback.
 *
 * @see Property
 * @see DisplayPropertySpec
 * @see property_factory.hpp
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/properties/property.hpp"

namespace autoviz {
namespace common {

/**
 * @class PropertyTreeBuilder
 * @brief Imperative API to grow a property tree under an owned root group.
 *
 * ## Typical use
 *
 * @code
 * PropertyTreeBuilder b([&](const std::string& k, const std::string& d) {
 *   return display->propertyValue(k, d);
 * });
 * Property* status = b.addGroup("Status", "Status");
 * b.addBool(status, "enabled", "Enabled", "true");
 * auto root = b.takeRoot();
 * @endcode
 */
class PropertyTreeBuilder {
 public:
  /**
   * @brief Lookup of current property values by key with default fallback.
   */
  using Getter =
      std::function<std::string(const std::string& key, const std::string& default_value)>;

  /**
   * @brief Constructs a builder with an empty root group.
   * @param getter Value lookup used when adding leaves.
   */
  explicit PropertyTreeBuilder(Getter getter);

  /**
   * @brief Non-owning pointer to the root group.
   * @return Root property (valid until @ref takeRoot()).
   */
  Property* root() { return root_.get(); }

  /**
   * @brief Adds a named subgroup under the root (or nested via return value).
   *
   * @param name Stable group key.
   * @param label UI label.
   * @param description Optional tooltip.
   * @return Pointer to the new group (for passing as parent to add*).
   */
  Property* addGroup(const std::string& name, const std::string& label,
                     const std::string& description = {});

  /**
   * @brief Adds a leaf under @p parent from a full @ref DisplayPropertySpec.
   *
   * @param parent Parent group (non-null).
   * @param spec Property schema (kind, options, defaults).
   */
  void addSpec(Property* parent, const DisplayPropertySpec& spec);

  /**
   * @brief Adds a string leaf.
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default if getter has no value.
   */
  void addString(Property* parent, const std::string& key,
                 const std::string& label, const std::string& default_value);

  /**
   * @brief Adds a bool leaf.
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default string (@c "true"/@c "false").
   */
  void addBool(Property* parent, const std::string& key,
               const std::string& label, const std::string& default_value);

  /**
   * @brief Adds a float leaf.
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default numeric string.
   */
  void addFloat(Property* parent, const std::string& key,
                const std::string& label, const std::string& default_value);

  /**
   * @brief Adds an int leaf.
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default numeric string.
   */
  void addInt(Property* parent, const std::string& key, const std::string& label,
              const std::string& default_value);

  /**
   * @brief Adds a color leaf (@c "R;G;B").
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default color string.
   */
  void addColor(Property* parent, const std::string& key,
                const std::string& label, const std::string& default_value);

  /**
   * @brief Adds a filesystem path leaf.
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default path string.
   */
  void addPath(Property* parent, const std::string& key,
               const std::string& label, const std::string& default_value);

  /**
   * @brief Adds a channel-name leaf (combo editor).
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default channel name.
   */
  void addChannel(Property* parent, const std::string& key,
                  const std::string& label, const std::string& default_value);

  /**
   * @brief Adds an enum leaf with fixed options.
   *
   * @param parent Parent group.
   * @param key Property key.
   * @param label UI label.
   * @param default_value Default selected option.
   * @param options Allowed values.
   */
  void addEnum(Property* parent, const std::string& key,
               const std::string& label, const std::string& default_value,
               std::vector<std::string> options);

  /**
   * @brief Releases ownership of the completed tree.
   * @return Root property unique_ptr (builder becomes empty).
   */
  std::unique_ptr<Property> takeRoot() { return std::move(root_); }

 private:
  /**
   * @brief Resolves a value via @c getter_.
   *
   * @param key Property key.
   * @param default_value Fallback.
   * @return Current or default value string.
   */
  std::string value(const std::string& key, const std::string& default_value) const;

  Getter getter_;                     /**< Value lookup. */
  std::unique_ptr<Property> root_;    /**< Owned tree root. */
};

}  // namespace common
}  // namespace autoviz
