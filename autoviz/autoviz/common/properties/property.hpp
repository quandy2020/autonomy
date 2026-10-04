/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * rviz_common::properties::Property — hierarchical display settings (zero ROS).
 *****************************************************************************/

/**
 * @file property.hpp
 * @brief Hierarchical property tree nodes for display / tool settings.
 *
 * Mirrors @c rviz_common::properties::Property without ROS: a tree of group
 * and leaf nodes (string, bool, float, int, color, enum) that sync with
 * @ref DisplayPropertyMap via @ref property_factory.hpp.
 *
 * @see PropertyTreeBuilder
 * @see PropertyFactory
 * @see DisplayPropertySpec
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/display_property.hpp"

namespace autoviz {
namespace common {

/**
 * @class Property
 * @brief Base node: group (non-empty children) or leaf (value-bearing subclass).
 *
 * Groups organize the Displays tree; leaves expose @ref valueString /
 * @ref setValueString and an optional @ref editorKind for UI editors.
 */
class Property {
 public:
  /**
   * @brief Callback invoked when a property value changes.
   * Argument is the property that changed (usually @c this).
   */
  using ChangedCallback = std::function<void(Property*)>;

  /**
   * @brief Constructs a property node and optionally attaches to a parent.
   *
   * @param name Stable key (matches @ref DisplayPropertyMap keys for leaves).
   * @param label Human-readable label.
   * @param parent Optional parent group; when non-null, this node is added
   *        as a child of @p parent.
   */
  Property(std::string name, std::string label, Property* parent = nullptr);

  virtual ~Property() = default;

  /**
   * @brief Stable property name / key.
   * @return Name string.
   */
  const std::string& name() const { return name_; }

  /**
   * @brief UI label.
   * @return Label string.
   */
  const std::string& label() const { return label_; }

  /**
   * @brief Optional longer description / tooltip.
   * @return Description string (may be empty).
   */
  const std::string& description() const { return description_; }

  /**
   * @brief Sets the description / tooltip text.
   * @param description New description (moved).
   */
  void setDescription(std::string description) { description_ = std::move(description); }

  /**
   * @brief Parent group, or @c nullptr for the root.
   * @return Parent pointer.
   */
  Property* parent() const { return parent_; }

  /**
   * @brief Child properties (owned).
   * @return Const reference to the child list.
   */
  const std::vector<std::unique_ptr<Property>>& children() const { return children_; }

  /**
   * @brief Whether this node is a group (has children).
   * @return @c true if @c children_ is non-empty.
   */
  bool isGroup() const { return !children_.empty(); }

  /**
   * @brief Takes ownership of @p child and appends it.
   *
   * @param child Child node (must be non-null).
   * @return Raw pointer to the stored child.
   */
  Property* addChild(std::unique_ptr<Property> child);

  /**
   * @brief Finds a direct child by name.
   *
   * @param name Child @ref name().
   * @return Matching child, or @c nullptr.
   */
  Property* findChild(const std::string& name) const;

  /**
   * @brief Depth-first search for a leaf with the given name.
   *
   * @param name Leaf @ref name().
   * @return Matching leaf, or @c nullptr.
   */
  Property* findLeaf(const std::string& name) const;

  /**
   * @brief Registers a change notification callback.
   * @param callback Invoked from @ref notifyChanged().
   */
  void setChangedCallback(ChangedCallback callback);

  /**
   * @brief String form of the value (empty for pure groups).
   * @return Value string.
   */
  virtual std::string valueString() const;

  /**
   * @brief Parses and stores a value from its string form.
   * @param value New value text.
   */
  virtual void setValueString(const std::string& value);

  /**
   * @brief Editor kind hint for the Displays panel.
   * @return @ref DisplayPropertyKind (default @c kAuto).
   */
  virtual DisplayPropertyKind editorKind() const { return DisplayPropertyKind::kAuto; }

  /**
   * @brief Enum options when this property is an enumeration.
   * @return Option list (empty for non-enums).
   */
  virtual std::vector<std::string> enumOptions() const { return {}; }

 protected:
  /**
   * @brief Invokes @c changed_callback_ if set.
   */
  void notifyChanged();

  std::string name_;                          /**< Stable key. */
  std::string label_;                         /**< UI label. */
  std::string description_;                   /**< Tooltip / help. */
  Property* parent_ = nullptr;                /**< Non-owning parent. */
  std::vector<std::unique_ptr<Property>> children_; /**< Owned children. */
  ChangedCallback changed_callback_;          /**< Optional change hook. */
};

/**
 * @class StringProperty
 * @brief Leaf property storing an arbitrary string.
 */
class StringProperty : public Property {
 public:
  /**
   * @brief Constructs a string leaf.
   *
   * @param name Property key.
   * @param label UI label.
   * @param value Initial value.
   * @param parent Optional parent group.
   */
  StringProperty(std::string name, std::string label, std::string value = {},
                 Property* parent = nullptr);

  /** @copydoc Property::valueString */
  std::string valueString() const override;

  /** @copydoc Property::setValueString */
  void setValueString(const std::string& value) override;

 private:
  std::string value_; /**< Stored string. */
};

/**
 * @class BoolProperty
 * @brief Leaf property storing a boolean (@c "true"/@c "false" text).
 */
class BoolProperty : public Property {
 public:
  /**
   * @brief Constructs a bool leaf.
   *
   * @param name Property key.
   * @param label UI label.
   * @param value Initial value.
   * @param parent Optional parent group.
   */
  BoolProperty(std::string name, std::string label, bool value = false,
               Property* parent = nullptr);

  /** @copydoc Property::valueString */
  std::string valueString() const override;

  /** @copydoc Property::setValueString */
  void setValueString(const std::string& value) override;

  /**
   * @brief Editor kind (auto / checkbox via Displays panel).
   * @return @ref DisplayPropertyKind::kAuto.
   */
  DisplayPropertyKind editorKind() const override { return DisplayPropertyKind::kAuto; }

 private:
  bool value_ = false; /**< Stored bool. */
};

/**
 * @class FloatProperty
 * @brief Leaf property storing a floating-point number.
 */
class FloatProperty : public Property {
 public:
  /**
   * @brief Constructs a float leaf.
   *
   * @param name Property key.
   * @param label UI label.
   * @param value Initial value.
   * @param parent Optional parent group.
   */
  FloatProperty(std::string name, std::string label, float value = 0.f,
                Property* parent = nullptr);

  /** @copydoc Property::valueString */
  std::string valueString() const override;

  /** @copydoc Property::setValueString */
  void setValueString(const std::string& value) override;

 private:
  float value_ = 0.f; /**< Stored float. */
};

/**
 * @class IntProperty
 * @brief Leaf property storing an integer.
 */
class IntProperty : public Property {
 public:
  /**
   * @brief Constructs an int leaf.
   *
   * @param name Property key.
   * @param label UI label.
   * @param value Initial value.
   * @param parent Optional parent group.
   */
  IntProperty(std::string name, std::string label, int value = 0,
              Property* parent = nullptr);

  /** @copydoc Property::valueString */
  std::string valueString() const override;

  /** @copydoc Property::setValueString */
  void setValueString(const std::string& value) override;

 private:
  int value_ = 0; /**< Stored int. */
};

/**
 * @class ColorProperty
 * @brief Leaf property storing an @c "R;G;B" color string.
 */
class ColorProperty : public Property {
 public:
  /**
   * @brief Constructs a color leaf.
   *
   * @param name Property key.
   * @param label UI label.
   * @param value Initial @c "R;G;B" string.
   * @param parent Optional parent group.
   */
  ColorProperty(std::string name, std::string label, std::string value = "200;200;200",
                Property* parent = nullptr);

  /** @copydoc Property::valueString */
  std::string valueString() const override;

  /** @copydoc Property::setValueString */
  void setValueString(const std::string& value) override;

  /**
   * @brief Requests a color picker editor.
   * @return @ref DisplayPropertyKind::kColor.
   */
  DisplayPropertyKind editorKind() const override { return DisplayPropertyKind::kColor; }

 private:
  std::string value_; /**< Stored color text. */
};

/**
 * @class EnumProperty
 * @brief Leaf property storing one of a fixed set of string options.
 */
class EnumProperty : public Property {
 public:
  /**
   * @brief Constructs an enum leaf.
   *
   * @param name Property key.
   * @param label UI label.
   * @param value Initial selected option.
   * @param options Allowed values for the combo box.
   * @param parent Optional parent group.
   */
  EnumProperty(std::string name, std::string label, std::string value,
               std::vector<std::string> options, Property* parent = nullptr);

  /** @copydoc Property::valueString */
  std::string valueString() const override;

  /** @copydoc Property::setValueString */
  void setValueString(const std::string& value) override;

  /**
   * @brief Returns the allowed option list.
   * @return Copy of @c options_.
   */
  std::vector<std::string> enumOptions() const override { return options_; }

 private:
  std::string value_;                  /**< Selected option. */
  std::vector<std::string> options_;   /**< Allowed values. */
};

}  // namespace common
}  // namespace autoviz
