/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPointer>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QHideEvent;
class QMouseEvent;
class QPropertyAnimation;
class QShowEvent;

namespace autoviz::widget {

// Modal, parent-bound panel for settings, inspectors, and secondary workflows.
class Drawer final : public QWidget {
  Q_OBJECT
  Q_PROPERTY(qreal progress READ progress WRITE SetProgress)

 public:
  enum class Placement { kLeft, kRight, kTop, kBottom };

  explicit Drawer(QWidget* parent = nullptr);
  ~Drawer() override;

  Placement placement() const noexcept { return placement_; }
  void SetPlacement(Placement placement);
  int panel_extent() const noexcept { return panel_extent_; }
  void SetPanelExtent(int extent);
  bool dismiss_on_scrim_click() const noexcept {
    return dismiss_on_scrim_click_;
  }
  void SetDismissOnScrimClick(bool enabled);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QWidget* content_widget() const noexcept { return content_; }
  // Transfers ownership. Replacing content schedules the old widget for
  // deletion; call TakeContentWidget() first to retain it.
  void SetContentWidget(QWidget* content);
  QWidget* TakeContentWidget();

  bool is_open() const noexcept { return isVisible() && !closing_; }
  void Show();
  void Dismiss();

 signals:
  void Opened();
  void Closed();

 protected:
  bool event(QEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;
  void hideEvent(QHideEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void showEvent(QShowEvent* event) override;

 private:
  qreal progress() const noexcept { return progress_; }
  void SetProgress(qreal progress);
  QRectF PanelRect() const;
  void SyncToParent();
  void UpdateContentGeometry();
  bool AnimationsEnabled() const;

  QPropertyAnimation* animation_ = nullptr;
  QPointer<QWidget> observed_parent_;
  QWidget* content_ = nullptr;
  Placement placement_ = Placement::kRight;
  style::Tone tone_ = style::Tone::Shell;
  qreal progress_ = 0.0;
  int panel_extent_ = 320;
  bool dismiss_on_scrim_click_ = true;
  bool closing_ = false;
};

}  // namespace autoviz::widget
