/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/vector3_editor.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QVariant>
#include <array>

#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/number_field.hpp"

namespace autoviz::widget {

Vector3Editor::Vector3Editor(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizVector3Editor"));
  setProperty("autovizRole", QStringLiteral("vector3Editor"));
  setAccessibleName(tr("Three dimensional vector"));

  auto* layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(GetThemeTokens(tone_).spacing_sm);

  x_field_ = new NumberField(this);
  y_field_ = new NumberField(this);
  z_field_ = new NumberField(this);
  const std::array<NumberField*, 3> fields = {x_field_, y_field_, z_field_};
  const std::array<const char*, 3> labels = {"X", "Y", "Z"};

  for (std::size_t index = 0; index < fields.size(); ++index) {
    auto* label = new QLabel(QString::fromLatin1(labels[index]), this);
    label->setBuddy(fields[index]);
    fields[index]->setAccessibleName(
        tr("%1 coordinate").arg(QString::fromLatin1(labels[index])));
    layout->addWidget(label);
    layout->addWidget(fields[index], 1);
    connect(fields[index], qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &Vector3Editor::EmitValueChanged);
    connect(fields[index], &QDoubleSpinBox::editingFinished, this,
            &Vector3Editor::EditingFinished);
  }
}

QVector3D Vector3Editor::value() const {
  return QVector3D(static_cast<float>(x_field_->value()),
                   static_cast<float>(y_field_->value()),
                   static_cast<float>(z_field_->value()));
}

void Vector3Editor::SetValue(const QVector3D& value) {
  if (this->value() == value) {
    return;
  }
  const QSignalBlocker block_x(x_field_);
  const QSignalBlocker block_y(y_field_);
  const QSignalBlocker block_z(z_field_);
  x_field_->setValue(value.x());
  y_field_->setValue(value.y());
  z_field_->setValue(value.z());
  emit ValueChanged(this->value());
}

void Vector3Editor::SetRange(double minimum, double maximum) {
  const QVector3D previous = value();
  const QSignalBlocker block_x(x_field_);
  const QSignalBlocker block_y(y_field_);
  const QSignalBlocker block_z(z_field_);
  x_field_->setRange(minimum, maximum);
  y_field_->setRange(minimum, maximum);
  z_field_->setRange(minimum, maximum);
  if (previous != value()) {
    emit ValueChanged(value());
  }
}

void Vector3Editor::SetDecimals(int decimals) {
  const QVector3D previous = value();
  const QSignalBlocker block_x(x_field_);
  const QSignalBlocker block_y(y_field_);
  const QSignalBlocker block_z(z_field_);
  x_field_->setDecimals(decimals);
  y_field_->setDecimals(decimals);
  z_field_->setDecimals(decimals);
  if (previous != value()) {
    emit ValueChanged(value());
  }
}

void Vector3Editor::SetSingleStep(double step) {
  x_field_->setSingleStep(step);
  y_field_->setSingleStep(step);
  z_field_->setSingleStep(step);
}

void Vector3Editor::SetSuffix(const QString& suffix) {
  x_field_->setSuffix(suffix);
  y_field_->setSuffix(suffix);
  z_field_->setSuffix(suffix);
}

void Vector3Editor::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  x_field_->SetTone(tone);
  y_field_->SetTone(tone);
  z_field_->SetTone(tone);
  ApplyWidgetTheme(this, tone, true);
}

void Vector3Editor::EmitValueChanged() { emit ValueChanged(value()); }

}  // namespace autoviz::widget
