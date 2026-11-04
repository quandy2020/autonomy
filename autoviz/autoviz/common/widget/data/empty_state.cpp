/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/empty_state.hpp"

#include <QEvent>
#include <QLabel>
#include <QPainter>
#include <QPalette>
#include <QPushButton>
#include <QStyle>
#include <QVBoxLayout>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz {
namespace widget {

EmptyState::EmptyState(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizEmptyState"));
  setProperty("autovizRole", QStringLiteral("emptyState"));
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(24, 24, 24, 24);
  layout->setSpacing(8);
  layout->addStretch(1);

  icon_label_ = new QLabel(this);
  icon_label_->setFixedSize(48, 48);
  icon_label_->setAlignment(Qt::AlignCenter);
  icon_label_->setAccessibleName(tr("Empty view"));
  layout->addWidget(icon_label_, 0, Qt::AlignHCenter);

  title_label_ = new QLabel(this);
  title_label_->setTextFormat(Qt::PlainText);
  title_label_->setWordWrap(true);
  title_label_->setVisible(false);
  title_label_->setAlignment(Qt::AlignCenter);
  QFont title_font = title_label_->font();
  title_font.setBold(true);
  title_font.setPointSizeF(title_font.pointSizeF() + 1.0);
  title_label_->setFont(title_font);
  layout->addWidget(title_label_);

  description_label_ = new QLabel(this);
  description_label_->setTextFormat(Qt::PlainText);
  description_label_->setWordWrap(true);
  description_label_->setVisible(false);
  description_label_->setAlignment(Qt::AlignCenter);
  description_label_->setTextInteractionFlags(Qt::TextSelectableByMouse |
                                              Qt::TextSelectableByKeyboard);
  layout->addWidget(description_label_);

  action_button_ = new QPushButton(this);
  action_button_->setObjectName(QStringLiteral("AutovizEmptyStateAction"));
  action_button_->setVisible(false);
  connect(action_button_, &QPushButton::clicked, this,
          &EmptyState::ActionTriggered);
  layout->addWidget(action_button_, 0, Qt::AlignHCenter);
  layout->addStretch(1);

  icon_ = style()->standardIcon(QStyle::SP_FileDialogContentsView);
  ApplyTheme();
  UpdateAccessibility();
}

QIcon EmptyState::icon() const { return icon_; }

void EmptyState::SetIcon(const QIcon& icon) {
  icon_ = icon;
  uses_default_icon_ = false;
  UpdateIconPixmap();
}

QString EmptyState::title() const { return title_label_->text(); }

void EmptyState::SetTitle(const QString& title) {
  title_label_->setText(title);
  title_label_->setVisible(!title.isEmpty());
  UpdateAccessibility();
}

QString EmptyState::description() const { return description_label_->text(); }

void EmptyState::SetDescription(const QString& description) {
  description_label_->setText(description);
  description_label_->setVisible(!description.isEmpty());
  UpdateAccessibility();
}

QString EmptyState::action_text() const { return action_button_->text(); }

void EmptyState::SetActionText(const QString& text) {
  action_button_->setText(text);
  action_button_->setVisible(!text.isEmpty());
  action_button_->setAccessibleName(text);
  updateGeometry();
}

void EmptyState::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyTheme();
}

QSize EmptyState::sizeHint() const { return QSize(360, 240); }

void EmptyState::changeEvent(QEvent* event) {
  if (event->type() == QEvent::StyleChange && uses_default_icon_) {
    icon_ = style()->standardIcon(QStyle::SP_FileDialogContentsView);
  }
  if (event->type() == QEvent::StyleChange ||
      event->type() == QEvent::PaletteChange ||
      event->type() == QEvent::EnabledChange ||
      event->type() == QEvent::ScreenChangeInternal) {
    UpdateIconPixmap();
  }
  QWidget::changeEvent(event);
}

void EmptyState::ApplyTheme() {
  ApplyWidgetTheme(this, tone_);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);

  QPalette title_palette = title_label_->palette();
  title_palette.setColor(QPalette::WindowText, tokens.text);
  title_palette.setColor(QPalette::Text, tokens.text);
  title_palette.setColor(QPalette::Disabled, QPalette::WindowText,
                         tokens.disabled);
  title_palette.setColor(QPalette::Disabled, QPalette::Text, tokens.disabled);
  title_label_->setPalette(title_palette);

  QPalette description_palette = description_label_->palette();
  description_palette.setColor(QPalette::WindowText, tokens.text_muted);
  description_palette.setColor(QPalette::Text, tokens.text_muted);
  description_palette.setColor(QPalette::Disabled, QPalette::WindowText,
                               tokens.disabled);
  description_palette.setColor(QPalette::Disabled, QPalette::Text,
                               tokens.disabled);
  description_label_->setPalette(description_palette);
  UpdateIconPixmap();
  update();
}

void EmptyState::UpdateIconPixmap() {
  QPixmap pixmap = icon_.pixmap(icon_label_->size(), devicePixelRatioF(),
                                isEnabled() ? QIcon::Normal : QIcon::Disabled);
  if (!pixmap.isNull()) {
    QPainter painter(&pixmap);
    painter.setCompositionMode(QPainter::CompositionMode_SourceIn);
    const ThemeTokens& tokens = GetThemeTokens(this, tone_);
    painter.fillRect(pixmap.rect(),
                     isEnabled() ? tokens.text_muted : tokens.disabled);
  }
  icon_label_->setPixmap(pixmap);
}

void EmptyState::UpdateAccessibility() {
  QString text = title_label_->text();
  const QString description = description_label_->text();
  if (!description.isEmpty()) {
    if (!text.isEmpty()) {
      text += QStringLiteral(". ");
    }
    text += description;
  }
  setAccessibleName(text);
  setAccessibleDescription(text);
  updateGeometry();
}

}  // namespace widget
}  // namespace autoviz
