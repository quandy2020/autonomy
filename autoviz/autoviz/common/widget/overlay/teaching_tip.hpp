/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QObject>
#include <QPointer>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"
#include "autoviz/common/widget/overlay/popover.hpp"

class QLabel;
class QWidget;

namespace autoviz::widget {

class Button;

// Anchored explanatory callout with optional primary and dismiss actions.
class TeachingTip final : public QObject {
  Q_OBJECT

 public:
  explicit TeachingTip(QWidget* parent = nullptr);
  ~TeachingTip() override;

  QString title() const;
  void SetTitle(const QString& title);
  QString message() const;
  void SetMessage(const QString& message);
  QString action_text() const;
  void SetActionText(const QString& text);
  QString dismiss_text() const;
  void SetDismissText(const QString& text);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  void ShowFor(QWidget* anchor,
               Popover::Placement placement = Popover::Placement::kBottomStart);
  void Dismiss();

 signals:
  void ActionTriggered();
  void Dismissed();

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  void UpdateAccessibility();

  QPointer<Popover> popover_;
  QWidget* content_ = nullptr;
  QLabel* title_label_ = nullptr;
  QLabel* message_label_ = nullptr;
  Button* action_button_ = nullptr;
  Button* dismiss_button_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  bool session_visible_ = false;
};

}  // namespace autoviz::widget
