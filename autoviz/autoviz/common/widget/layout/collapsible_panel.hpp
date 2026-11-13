/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/widget/layout/surface.hpp"

class QEvent;
class QHideEvent;
class QPaintEvent;
class QPropertyAnimation;
class QVBoxLayout;

namespace autoviz::widget {

class Button;

// Accessible disclosure container for settings and inspector sections.
class CollapsiblePanel final : public Surface {
  Q_OBJECT

 public:
  explicit CollapsiblePanel(QWidget* parent = nullptr);

  QString title() const;
  void SetTitle(const QString& title);
  bool is_expanded() const noexcept { return expanded_; }
  void SetExpanded(bool expanded);
  bool animation_enabled() const noexcept { return animation_enabled_; }
  void SetAnimationEnabled(bool enabled);
  void SetTone(style::Tone tone);

  QWidget* content_widget() const noexcept { return content_; }
  // Transfers ownership. Replacing content schedules the old widget for
  // deletion; call TakeContentWidget() first to retain it.
  void SetContentWidget(QWidget* content);
  QWidget* TakeContentWidget();

 signals:
  void ExpandedChanged(bool expanded);

 protected:
  void changeEvent(QEvent* event) override;
  void hideEvent(QHideEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  void UpdateHeader();
  void StartTransition();
  void FinishTransition();
  int ExpandedHeight() const;

  Button* header_button_ = nullptr;
  QWidget* content_host_ = nullptr;
  QWidget* content_ = nullptr;
  QVBoxLayout* content_layout_ = nullptr;
  QPropertyAnimation* animation_ = nullptr;
  bool expanded_ = true;
  bool animation_enabled_ = true;
};

}  // namespace autoviz::widget
