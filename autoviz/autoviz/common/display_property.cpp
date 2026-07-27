/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/display_property.hpp"

#include <QStringList>

namespace autoviz {
namespace common {

QColor ParseColorProperty(const std::string& value, const QColor& fallback) {
  const QStringList parts =
      QString::fromStdString(value).split(';', Qt::SkipEmptyParts);
  if (parts.size() < 3) {
    return fallback;
  }
  return QColor(parts[0].toInt(), parts[1].toInt(), parts[2].toInt());
}

float ParseFloatProperty(const std::string& value, float fallback) {
  bool ok = false;
  const float parsed = QString::fromStdString(value).toFloat(&ok);
  return ok ? parsed : fallback;
}

int ParseIntProperty(const std::string& value, int fallback) {
  bool ok = false;
  const int parsed = QString::fromStdString(value).toInt(&ok);
  return ok ? parsed : fallback;
}

QVector3D ParseVector3Property(const std::string& value,
                               const QVector3D& fallback) {
  const QStringList parts =
      QString::fromStdString(value).split(';', Qt::SkipEmptyParts);
  if (parts.size() < 3) {
    return fallback;
  }
  bool ok_x = false;
  bool ok_y = false;
  bool ok_z = false;
  const float x = parts[0].trimmed().toFloat(&ok_x);
  const float y = parts[1].trimmed().toFloat(&ok_y);
  const float z = parts[2].trimmed().toFloat(&ok_z);
  if (!ok_x || !ok_y || !ok_z) {
    return fallback;
  }
  return QVector3D(x, y, z);
}

bool ParseBoolProperty(const std::string& value, bool fallback) {
  const QString lowered = QString::fromStdString(value).trimmed().toLower();
  if (lowered == QLatin1String("true") || lowered == QLatin1String("1") ||
      lowered == QLatin1String("yes")) {
    return true;
  }
  if (lowered == QLatin1String("false") || lowered == QLatin1String("0") ||
      lowered == QLatin1String("no")) {
    return false;
  }
  return fallback;
}

std::string FormatColorProperty(const QColor& color) {
  return std::to_string(color.red()) + ";" + std::to_string(color.green()) +
         ";" + std::to_string(color.blue());
}

std::string FormatVector3Property(const QVector3D& vector) {
  return std::to_string(vector.x()) + ";" + std::to_string(vector.y()) + ";" +
         std::to_string(vector.z());
}

}  // namespace common
}  // namespace autoviz
