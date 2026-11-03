/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QHBoxLayout;
class QLabel;

namespace autoviz::widget {

class NumberField;

// Compact X/Y/Z editor for poses, dimensions, velocities, and map parameters.
class Vector3Editor final : public QWidget {
  Q_OBJECT

 public:
  explicit Vector3Editor(QWidget* parent = nullptr);

  QVector3D value() const;
  void SetValue(const QVector3D& value);
  void SetRange(double minimum, double maximum);
  void SetDecimals(int decimals);
  void SetSingleStep(double step);
  void SetSuffix(const QString& suffix);
  void SetTone(style::Tone tone);

 signals:
  void ValueChanged(const QVector3D& value);
  void EditingFinished();

 private:
  void EmitValueChanged();

  NumberField* x_field_ = nullptr;
  NumberField* y_field_ = nullptr;
  NumberField* z_field_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
