/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QIcon>
#include <QString>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QLabel;
class QPushButton;

namespace autoviz {
namespace widget {

// Standard placeholder for a data view with no rows or unavailable content.
class EmptyState : public QWidget {
  Q_OBJECT

 public:
  explicit EmptyState(QWidget* parent = nullptr);

  QIcon icon() const;
  void SetIcon(const QIcon& icon);

  QString title() const;
  void SetTitle(const QString& title);

  QString description() const;
  void SetDescription(const QString& description);

  QString action_text() const;
  void SetActionText(const QString& text);

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;

 signals:
  void ActionTriggered();

 protected:
  void changeEvent(QEvent* event) override;

 private:
  void ApplyTheme();
  void UpdateIconPixmap();
  void UpdateAccessibility();

  QIcon icon_;
  QLabel* icon_label_ = nullptr;
  QLabel* title_label_ = nullptr;
  QLabel* description_label_ = nullptr;
  QPushButton* action_button_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  bool uses_default_icon_ = true;
};

}  // namespace widget
}  // namespace autoviz
