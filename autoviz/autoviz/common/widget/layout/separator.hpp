/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

namespace autoviz::widget {

class Separator final : public QWidget {
 public:
  explicit Separator(Qt::Orientation orientation = Qt::Horizontal,
                     QWidget* parent = nullptr);

  Qt::Orientation orientation() const noexcept { return orientation_; }
  void SetOrientation(Qt::Orientation orientation);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  Qt::Orientation orientation_ = Qt::Horizontal;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
