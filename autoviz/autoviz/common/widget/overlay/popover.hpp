/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QFrame>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QVBoxLayout;

namespace autoviz::widget {

class Popover final : public QFrame {
 public:
  enum class Placement {
    kBottomStart,
    kBottomEnd,
    kTopStart,
    kTopEnd,
  };

  explicit Popover(QWidget* parent = nullptr);

  QWidget* content_widget() const noexcept { return content_; }
  // Transfers ownership to the popover. Replacing content schedules the old
  // widget for deletion; call TakeContentWidget() first to keep it.
  void SetContentWidget(QWidget* content);
  QWidget* TakeContentWidget();
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  void ShowFor(QWidget* anchor, Placement placement = Placement::kBottomStart);
  void Dismiss();

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  QPoint PositionFor(const QRect& anchor, Placement placement) const;

  QVBoxLayout* layout_ = nullptr;
  QWidget* content_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
