/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractButton>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QKeyEvent;
class QMouseEvent;
class QPaintEvent;

namespace autoviz::widget {

// Accessible integer rating input. Stars are vector paths and do not depend on
// icon fonts or third-party assets.
class Rating final : public QAbstractButton {
  Q_OBJECT
  Q_PROPERTY(int value READ value WRITE SetValue NOTIFY ValueChanged)

 public:
  explicit Rating(QWidget* parent = nullptr);

  int value() const noexcept { return value_; }
  void SetValue(int value);
  int maximum() const noexcept { return maximum_; }
  void SetMaximum(int maximum);
  bool clear_enabled() const noexcept { return clear_enabled_; }
  void SetClearEnabled(bool enabled);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void ValueChanged(int value);

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void leaveEvent(QEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  int ValueAtPosition(qreal x) const;
  void UpdateAccessibility();

  style::Tone tone_ = style::Tone::Shell;
  int value_ = 0;
  int maximum_ = 5;
  int hover_value_ = -1;
  bool clear_enabled_ = true;
};

}  // namespace autoviz::widget
