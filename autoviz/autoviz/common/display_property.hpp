/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_property.hpp
 * @brief Property map types, editor kinds, specs, and parse/format helpers.
 *
 * Display and tool settings are stored as string key/value maps
 * (@ref DisplayPropertyMap). @ref DisplayPropertySpec describes how the
 * Displays / Tools panels should edit each key (combo, color, nested tree).
 *
 * @see properties/property.hpp
 * @see properties/property_factory.hpp
 * @see display::Display
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

namespace autoviz {
namespace common {

/**
 * @brief String key → string value map for display/tool properties.
 *
 * Values are always stored as text; typed accessors use the Parse* and
 * Format* helpers below.
 */
using DisplayPropertyMap = std::map<std::string, std::string>;

/**
 * @enum DisplayPropertyKind
 * @brief Editor widget hint for a property row in the Displays panel.
 */
enum class DisplayPropertyKind {
  kAuto,     /**< Infer editor from value / options (default). */
  kColor,    /**< Color picker (@c "R;G;B" text). */
  kChannel,  /**< Channel name combo from the live channel list. */
  kPath,     /**< Filesystem path browser. */
  kCategory, /**< Non-editable group / category header. */
  kReadOnly, /**< Display-only value (not user-editable). */
  kRegex,    /**< Regex / pattern text field. */
  kInt,      /**< Integer spin / validated int field. */
};

/**
 * @struct DisplayPropertySpec
 * @brief Schema entry describing one editable property for a display or tool.
 *
 * Specs drive property-tree construction (@ref PropertyTreeBuilder) and the
 * Displays panel editors. Conditional visibility and nesting follow RViz
 * conventions.
 */
struct DisplayPropertySpec {
  /** Stable property key stored in @ref DisplayPropertyMap. */
  std::string key;

  /** Human-readable label shown in the UI. */
  std::string label;

  /** Default value string when the key is absent from the map. */
  std::string default_value;

  /**
   * Non-empty → QComboBox / enum editor listing these options.
   * Used with @ref DisplayPropertyKind::kAuto or explicit enum handling.
   */
  std::vector<std::string> options;

  /** Preferred editor kind (defaults to @ref DisplayPropertyKind::kAuto). */
  DisplayPropertyKind kind = DisplayPropertyKind::kAuto;

  /**
   * Conditional visibility: hide this row unless the sibling property named
   * @c visible_when_key equals one of the pipe-separated values in
   * @c visible_when_values (e.g. key=@c "color_transform",
   * values=@c "Intensity|AxisColor X"). Empty strings mean always visible.
   */
  std::string visible_when_key;

  /** Pipe-separated allowed values for @c visible_when_key (see above). */
  std::string visible_when_values;

  /**
   * Non-empty → nest this row under the property with the given key
   * (RViz-style parent/child tree).
   */
  std::string parent_key;

  /**
   * When @c true, expand into X/Y/Z child rows
   * (RViz @c VectorProperty equivalent).
   */
  bool expand_as_vector = false;
};

/**
 * @brief Parses a color property string into a @c QColor.
 *
 * Accepts @c "R;G;B" (0–255 components) and related formats used by Autoviz.
 *
 * @param value Raw property string.
 * @param fallback Color returned on parse failure.
 * @return Parsed color, or @p fallback.
 * @see FormatColorProperty()
 */
QColor ParseColorProperty(const std::string& value,
                          const QColor& fallback = QColor(200, 200, 200));

/**
 * @brief Parses a floating-point property string.
 *
 * @param value Raw property string.
 * @param fallback Value on failure.
 * @return Parsed float, or @p fallback.
 */
float ParseFloatProperty(const std::string& value, float fallback);

/**
 * @brief Parses an integer property string.
 *
 * @param value Raw property string.
 * @param fallback Value on failure.
 * @return Parsed int, or @p fallback.
 */
int ParseIntProperty(const std::string& value, int fallback);

/**
 * @brief Parses a boolean property string (@c "true"/@c "false", @c "1"/@c "0").
 *
 * @param value Raw property string.
 * @param fallback Value on failure.
 * @return Parsed bool, or @p fallback.
 */
bool ParseBoolProperty(const std::string& value, bool fallback);

/**
 * @brief Parses a semicolon-separated @c x;y;z vector (meters).
 *
 * @param value Raw property string.
 * @param fallback Vector on failure.
 * @return Parsed @c QVector3D, or @p fallback.
 * @see FormatVector3Property()
 */
QVector3D ParseVector3Property(const std::string& value,
                               const QVector3D& fallback = QVector3D());

/**
 * @brief Formats a @c QColor as an Autoviz color property string.
 *
 * @param color Color to format.
 * @return @c "R;G;B" string.
 * @see ParseColorProperty()
 */
std::string FormatColorProperty(const QColor& color);

/**
 * @brief Formats a float compactly (e.g. @c 0.01 not @c 0.0100 / @c 0.010000).
 *
 * @param value Number to format.
 * @return Compact decimal string.
 */
std::string FormatFloatProperty(double value);

/**
 * @brief Formats a vector as @c "x;y;z".
 *
 * @param vector Vector to format.
 * @return Semicolon-separated string.
 * @see ParseVector3Property()
 */
std::string FormatVector3Property(const QVector3D& vector);

}  // namespace common
}  // namespace autoviz
