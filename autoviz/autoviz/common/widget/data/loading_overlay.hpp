/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPointer>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QLabel;

namespace autoviz::widget {

class ProgressRing;

// Lightweight loading state for an existing view. The embedded progress ring
// stops its timer whenever the overlay or target is hidden.
class LoadingOverlay final : public QWidget {
  Q_OBJECT
  Q_PROPERTY(
      bool loading READ is_loading WRITE SetLoading NOTIFY LoadingChanged)
  Q_PROPERTY(QString message READ message WRITE SetMessage)

 public:
  explicit LoadingOverlay(QWidget* target = nullptr);
  ~LoadingOverlay() override;

  QWidget* target_widget() const noexcept { return target_.data(); }
  void AttachTo(QWidget* target);

  bool is_loading() const noexcept { return loading_; }
  QString message() const;
  void SetMessage(const QString& message);

  bool blocks_input() const noexcept { return blocks_input_; }
  void SetBlocksInput(bool blocks_input);

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 public slots:
  void SetLoading(bool loading);

 signals:
  void LoadingChanged(bool loading);

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private:
  void CapturePreviousFocus();
  void ReleaseInputFilter();
  void RestorePreviousFocus();
  void SyncGeometryAndVisibility();
  void SyncAnimation();
  void SyncInputFilter();

  QPointer<QWidget> target_;
  QPointer<QWidget> previous_focus_;
  ProgressRing* progress_ring_ = nullptr;
  QLabel* message_label_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  bool loading_ = false;
  bool blocks_input_ = true;
  bool application_filter_installed_ = false;
};

}  // namespace autoviz::widget
