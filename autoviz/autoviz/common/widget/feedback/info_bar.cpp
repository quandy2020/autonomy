/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/feedback/info_bar.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPointer>
#include <QPushButton>
#include <QStyle>
#include <QStyleOption>
#include <QVBoxLayout>
#include <algorithm>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz {
namespace widget {
namespace {

QStyle::StandardPixmap IconForLevel(FeedbackLevel level) {
  switch (level) {
    case FeedbackLevel::kInformation:
      return QStyle::SP_MessageBoxInformation;
    case FeedbackLevel::kSuccess:
      return QStyle::SP_DialogApplyButton;
    case FeedbackLevel::kWarning:
      return QStyle::SP_MessageBoxWarning;
    case FeedbackLevel::kError:
      return QStyle::SP_MessageBoxCritical;
  }
  return QStyle::SP_MessageBoxInformation;
}

QColor SemanticColor(FeedbackLevel level, style::Tone tone) {
  const ThemeTokens& tokens = GetThemeTokens(tone);
  switch (level) {
    case FeedbackLevel::kInformation:
      return tokens.accent;
    case FeedbackLevel::kSuccess:
      return tokens.success;
    case FeedbackLevel::kWarning:
      return tokens.warning;
    case FeedbackLevel::kError:
      return tokens.danger;
  }
  return tokens.accent;
}

QColor Blend(const QColor& foreground, const QColor& background, int alpha) {
  const int inverse = 255 - alpha;
  return QColor(
      (foreground.red() * alpha + background.red() * inverse) / 255,
      (foreground.green() * alpha + background.green() * inverse) / 255,
      (foreground.blue() * alpha + background.blue() * inverse) / 255);
}

}  // namespace

InfoBar::InfoBar(QWidget* parent) : QFrame(parent) { Initialize(); }

InfoBar::InfoBar(const QString& title, const QString& message,
                 FeedbackLevel level, QWidget* parent)
    : QFrame(parent), level_(level) {
  Initialize();
  SetTitle(title);
  SetMessage(message);
}

QString InfoBar::title() const { return title_label_->text(); }

void InfoBar::SetTitle(const QString& title) {
  title_label_->setText(title);
  title_label_->setVisible(!title.isEmpty());
  UpdatePresentation();
}

QString InfoBar::message() const { return message_label_->text(); }

void InfoBar::SetMessage(const QString& message) {
  message_label_->setText(message);
  message_label_->setVisible(!message.isEmpty());
  UpdatePresentation();
}

FeedbackLevel InfoBar::level() const { return level_; }

void InfoBar::SetLevel(FeedbackLevel level) {
  if (level_ == level) {
    return;
  }
  level_ = level;
  UpdateSemanticColors();
  UpdatePresentation();
}

void InfoBar::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyWidgetTheme(this, tone_, true);
  UpdateSemanticColors();
  UpdatePresentation();
}

bool InfoBar::is_closable() const { return closable_; }

void InfoBar::SetClosable(bool closable) {
  if (closable_ == closable) {
    return;
  }
  closable_ = closable;
  close_button_->setVisible(closable);
  updateGeometry();
}

QSize InfoBar::sizeHint() const {
  QSize hint = QFrame::sizeHint();
  hint.setWidth(std::max(hint.width(), 320));
  return hint;
}

void InfoBar::changeEvent(QEvent* event) {
  if (event->type() == QEvent::PaletteChange ||
      event->type() == QEvent::StyleChange ||
      event->type() == QEvent::ScreenChangeInternal) {
    UpdateSemanticColors();
    UpdatePresentation();
  }
  QFrame::changeEvent(event);
}

void InfoBar::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(border_color_, 1.0));
  painter.setBrush(background_color_);
  painter.drawRoundedRect(rect().adjusted(0, 0, -1, -1), 6.0, 6.0);
}

void InfoBar::Initialize() {
  setObjectName(QStringLiteral("AutovizInfoBar"));
  setProperty("autovizRole", QStringLiteral("infoBar"));
  setFrameStyle(QFrame::NoFrame);
  setAttribute(Qt::WA_OpaquePaintEvent, false);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

  auto* root = new QHBoxLayout(this);
  root->setContentsMargins(12, 10, 8, 10);
  root->setSpacing(10);

  icon_label_ = new QLabel(this);
  icon_label_->setFixedSize(20, 20);
  icon_label_->setScaledContents(true);
  icon_label_->setAccessibleName(tr("Notification type"));
  root->addWidget(icon_label_, 0, Qt::AlignTop);

  auto* text_layout = new QVBoxLayout;
  text_layout->setContentsMargins(0, 0, 0, 0);
  text_layout->setSpacing(2);

  title_label_ = new QLabel(this);
  title_label_->setTextFormat(Qt::PlainText);
  title_label_->setWordWrap(true);
  title_label_->setVisible(false);
  QFont title_font = title_label_->font();
  title_font.setBold(true);
  title_label_->setFont(title_font);
  text_layout->addWidget(title_label_);

  message_label_ = new QLabel(this);
  message_label_->setTextFormat(Qt::PlainText);
  message_label_->setWordWrap(true);
  message_label_->setVisible(false);
  message_label_->setTextInteractionFlags(Qt::TextSelectableByMouse |
                                          Qt::TextSelectableByKeyboard);
  text_layout->addWidget(message_label_);
  root->addLayout(text_layout, 1);

  close_button_ = new QPushButton(this);
  close_button_->setObjectName(QStringLiteral("AutovizInfoBarCloseButton"));
  close_button_->setFlat(true);
  close_button_->setIcon(style()->standardIcon(QStyle::SP_DialogCloseButton));
  close_button_->setFixedSize(28, 28);
  close_button_->setAccessibleName(tr("Close notification"));
  connect(close_button_, &QPushButton::clicked, this, [this]() {
    QPointer<InfoBar> guard(this);
    emit CloseRequested();
    if (guard != nullptr) {
      guard->hide();
    }
  });
  root->addWidget(close_button_, 0, Qt::AlignTop);

  ApplyWidgetTheme(this, tone_, true);
  UpdateSemanticColors();
  UpdatePresentation();
}

void InfoBar::UpdatePresentation() {
  const QIcon icon = style()->standardIcon(IconForLevel(level_));
  icon_label_->setPixmap(
      icon.pixmap(icon_label_->size(), devicePixelRatioF(),
                  isEnabled() ? QIcon::Normal : QIcon::Disabled));

  QString accessible_text = title_label_->text();
  const QString message = message_label_->text();
  if (!message.isEmpty()) {
    if (!accessible_text.isEmpty()) {
      accessible_text += QStringLiteral(". ");
    }
    accessible_text += message;
  }
  setAccessibleName(accessible_text);
  setAccessibleDescription(accessible_text);
  updateGeometry();
  update();
}

void InfoBar::UpdateSemanticColors() {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  border_color_ = SemanticColor(level_, tone_);
  background_color_ = Blend(border_color_, tokens.surface, 24);
}

}  // namespace widget
}  // namespace autoviz
