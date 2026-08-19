/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <map>
#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

namespace autoviz {
namespace common {

using DisplayPropertyMap = std::map<std::string, std::string>;

enum class DisplayPropertyKind {
  kAuto,
  kColor,
  kChannel,
  kPath,
  kCategory,
  kReadOnly,
  kRegex,
};

struct DisplayPropertySpec {
  std::string key;
  std::string label;
  std::string default_value;
  /** Non-empty → QComboBox in Displays panel. */
  std::vector<std::string> options;
  DisplayPropertyKind kind = DisplayPropertyKind::kAuto;
  /** Conditional visibility: hide this row unless the sibling property
   *  named `visible_when_key` equals one of the pipe-separated values in
   *  `visible_when_values` (e.g. key="color_transform", values="Intensity|AxisColor X").
   *  Empty strings mean always visible. */
  std::string visible_when_key;
  std::string visible_when_values;
};

QColor ParseColorProperty(const std::string& value,
                          const QColor& fallback = QColor(200, 200, 200));
float ParseFloatProperty(const std::string& value, float fallback);
int ParseIntProperty(const std::string& value, int fallback);
bool ParseBoolProperty(const std::string& value, bool fallback);
/** Semicolon-separated x;y;z (meters). */
QVector3D ParseVector3Property(const std::string& value,
                               const QVector3D& fallback = QVector3D());
std::string FormatColorProperty(const QColor& color);
std::string FormatVector3Property(const QVector3D& vector);

}  // namespace common
}  // namespace autoviz
