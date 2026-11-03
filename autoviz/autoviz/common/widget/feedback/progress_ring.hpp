/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QBasicTimer>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

namespace autoviz::widget {

class ProgressRing final : public QWidget {
 public:
  explicit ProgressRing(QWidget* parent = nullptr);

  int minimum() const noexcept { return minimum_; }
  int maximum() const noexcept { return maximum_; }
  int value() const noexcept { return value_; }
  void SetRange(int minimum, int maximum);
  void SetValue(int value);
  bool is_indeterminate() const noexcept { return indeterminate_; }
  void SetIndeterminate(bool indeterminate);
  int stroke_width() const noexcept { return stroke_width_; }
  void SetStrokeWidth(int width);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  void changeEvent(QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;
  void timerEvent(QTimerEvent* event) override;

 private:
  void SyncAnimation();
  void UpdateAccessibleValue();

  style::Tone tone_ = style::Tone::Shell;
  QBasicTimer timer_;
  int minimum_ = 0;
  int maximum_ = 100;
  int value_ = 0;
  int stroke_width_ = 3;
  int rotation_degrees_ = 0;
  bool indeterminate_ = false;
};

}  // namespace autoviz::widget
