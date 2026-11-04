/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/overlay/teaching_tip.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/button.hpp"

namespace autoviz::widget {

TeachingTip::TeachingTip(QWidget* parent)
    : QObject(parent), popover_(new Popover(parent)), content_(new QWidget) {
  content_->setObjectName(QStringLiteral("AutovizTeachingTip"));
  content_->setAccessibleName(tr("Teaching tip"));
  content_->setMinimumWidth(280);
  content_->setMaximumWidth(420);

  auto* root = new QVBoxLayout(content_);
  const ThemeTokens& tokens = GetThemeTokens(content_, tone_);
  root->setContentsMargins(tokens.spacing_sm, tokens.spacing_sm,
                           tokens.spacing_sm, tokens.spacing_sm);
  root->setSpacing(tokens.spacing_sm);

  title_label_ = new QLabel(content_);
  title_label_->setTextFormat(Qt::PlainText);
  title_label_->setWordWrap(true);
  title_label_->setVisible(false);
  QFont title_font = title_label_->font();
  title_font.setBold(true);
  title_label_->setFont(title_font);
  root->addWidget(title_label_);

  message_label_ = new QLabel(content_);
  message_label_->setTextFormat(Qt::PlainText);
  message_label_->setWordWrap(true);
  message_label_->setVisible(false);
  message_label_->setTextInteractionFlags(Qt::TextSelectableByMouse |
                                          Qt::TextSelectableByKeyboard);
  root->addWidget(message_label_);

  auto* actions = new QHBoxLayout;
  actions->setContentsMargins(0, 0, 0, 0);
  actions->setSpacing(tokens.spacing_sm);
  actions->addStretch(1);

  dismiss_button_ = new Button(tr("Got it"), content_);
  dismiss_button_->SetVariant(Button::Variant::kSubtle);
  connect(dismiss_button_, &QAbstractButton::clicked, this,
          &TeachingTip::Dismiss);
  actions->addWidget(dismiss_button_);

  action_button_ = new Button(content_);
  action_button_->SetVariant(Button::Variant::kPrimary);
  action_button_->setVisible(false);
  connect(action_button_, &QAbstractButton::clicked, this, [this]() {
    QPointer<TeachingTip> guard(this);
    emit ActionTriggered();
    if (guard != nullptr) {
      guard->Dismiss();
    }
  });
  actions->addWidget(action_button_);
  root->addLayout(actions);

  popover_->SetTone(tone_);
  popover_->SetContentWidget(content_);
  popover_->installEventFilter(this);
  SetTone(tone_);
  UpdateAccessibility();
}

TeachingTip::~TeachingTip() {
  if (popover_ != nullptr) {
    popover_->removeEventFilter(this);
    popover_->hide();
    popover_->deleteLater();
    popover_.clear();
  }
}

QString TeachingTip::title() const { return title_label_->text(); }

void TeachingTip::SetTitle(const QString& title) {
  title_label_->setText(title);
  title_label_->setVisible(!title.isEmpty());
  UpdateAccessibility();
}

QString TeachingTip::message() const { return message_label_->text(); }

void TeachingTip::SetMessage(const QString& message) {
  message_label_->setText(message);
  message_label_->setVisible(!message.isEmpty());
  UpdateAccessibility();
}

QString TeachingTip::action_text() const { return action_button_->text(); }

void TeachingTip::SetActionText(const QString& text) {
  action_button_->setText(text);
  action_button_->setAccessibleName(text);
  action_button_->setVisible(!text.isEmpty());
  content_->adjustSize();
}

QString TeachingTip::dismiss_text() const { return dismiss_button_->text(); }

void TeachingTip::SetDismissText(const QString& text) {
  dismiss_button_->setText(text);
  dismiss_button_->setAccessibleName(text);
  dismiss_button_->setVisible(!text.isEmpty());
  content_->adjustSize();
}

void TeachingTip::SetTone(style::Tone tone) {
  tone_ = tone;
  if (popover_ != nullptr) {
    popover_->SetTone(tone);
  }
  ApplyWidgetTheme(content_, tone, true);
  action_button_->SetTone(tone);
  dismiss_button_->SetTone(tone);
}

void TeachingTip::ShowFor(QWidget* anchor, Popover::Placement placement) {
  if (popover_ == nullptr || anchor == nullptr) {
    return;
  }
  session_visible_ = true;
  content_->adjustSize();
  popover_->ShowFor(anchor, placement);
}

void TeachingTip::Dismiss() {
  if (popover_ != nullptr) {
    popover_->Dismiss();
  }
}

bool TeachingTip::eventFilter(QObject* watched, QEvent* event) {
  if (watched == popover_ && event->type() == QEvent::Hide &&
      session_visible_) {
    session_visible_ = false;
    QPointer<TeachingTip> guard(this);
    QPointer<QObject> watched_guard(watched);
    emit Dismissed();
    if (guard == nullptr || watched_guard == nullptr) {
      return false;
    }
  }
  return QObject::eventFilter(watched, event);
}

void TeachingTip::UpdateAccessibility() {
  const QString accessible_text = title_label_->text().isEmpty()
                                      ? message_label_->text()
                                      : title_label_->text() +
                                            QStringLiteral(". ") +
                                            message_label_->text();
  content_->setAccessibleName(accessible_text);
  content_->setAccessibleDescription(accessible_text);
}

}  // namespace autoviz::widget
