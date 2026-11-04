/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/feedback/toast.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QLabel>
#include <QPaintEvent>
#include <QPainter>
#include <QPushButton>
#include <QShowEvent>
#include <QTimer>
#include <algorithm>

#include "autoviz/common/widget/feedback/feedback_accessibility.hpp"
#include "autoviz/common/widget/foundation/semantic_icon.hpp"
#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz {
namespace widget {
namespace {

internal::SemanticStatusIcon IconForLevel(FeedbackLevel level) {
  switch (level) {
    case FeedbackLevel::kInformation:
      return internal::SemanticStatusIcon::kInformation;
    case FeedbackLevel::kSuccess:
      return internal::SemanticStatusIcon::kSuccess;
    case FeedbackLevel::kWarning:
      return internal::SemanticStatusIcon::kWarning;
    case FeedbackLevel::kError:
      return internal::SemanticStatusIcon::kError;
  }
  return internal::SemanticStatusIcon::kInformation;
}

QColor SemanticColor(FeedbackLevel level, const ThemeTokens& tokens) {
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
  const int bounded = qBound(0, alpha, 255);
  const int inverse = 255 - bounded;
  return QColor(
      (foreground.red() * bounded + background.red() * inverse) / 255,
      (foreground.green() * bounded + background.green() * inverse) / 255,
      (foreground.blue() * bounded + background.blue() * inverse) / 255,
      (foreground.alpha() * bounded + background.alpha() * inverse) / 255);
}

}  // namespace

Toast::Toast(QWidget* parent) : QFrame(parent) {
  setObjectName(QStringLiteral("AutovizToast"));
  setProperty("autovizRole", QStringLiteral("toast"));
  setFrameShape(QFrame::NoFrame);
  setAttribute(Qt::WA_TranslucentBackground);
  setAutoFillBackground(false);
  setFocusPolicy(Qt::NoFocus);
  setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);

  auto* layout = new QHBoxLayout(this);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  layout->setContentsMargins(tokens.spacing_lg, tokens.spacing_sm,
                             tokens.spacing_sm, tokens.spacing_sm + 2);
  layout->setSpacing(tokens.spacing_sm);

  icon_label_ = new QLabel(this);
  icon_label_->setObjectName(QStringLiteral("AutovizToastLevelIcon"));
  icon_label_->setFixedSize(18, 18);
  icon_label_->setScaledContents(true);
  layout->addWidget(icon_label_);

  message_label_ = new QLabel(this);
  message_label_->setTextFormat(Qt::PlainText);
  message_label_->setWordWrap(true);
  message_label_->setTextInteractionFlags(Qt::TextSelectableByMouse |
                                          Qt::TextSelectableByKeyboard);
  layout->addWidget(message_label_, 1);

  close_button_ = new QPushButton(this);
  close_button_->setObjectName(QStringLiteral("AutovizToastCloseButton"));
  close_button_->setFlat(true);
  close_button_->setFixedSize(44, 44);
  close_button_->setIconSize(QSize(18, 18));
  close_button_->setAccessibleName(tr("Dismiss notification"));
  connect(close_button_, &QPushButton::clicked, this, &Toast::Dismiss);
  layout->addWidget(close_button_);

  dismiss_timer_ = new QTimer(this);
  dismiss_timer_->setSingleShot(true);
  connect(dismiss_timer_, &QTimer::timeout, this, &Toast::Dismiss);

  if (parent != nullptr) {
    parent->installEventFilter(this);
  }
  ApplyTheme();
  UpdateIcon();
}

QString Toast::message() const { return message_label_->text(); }

void Toast::SetMessage(const QString& message) {
  message_label_->setText(message);
  UpdateAccessibility();
  updateGeometry();
  if (isVisible()) {
    adjustSize();
    Reposition();
  }
}

FeedbackLevel Toast::level() const { return level_; }

void Toast::SetLevel(FeedbackLevel level) {
  if (level_ == level) {
    return;
  }
  level_ = level;
  UpdateIcon();
  update();
}

void Toast::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyTheme();
  UpdateIcon();
}

int Toast::duration_ms() const { return duration_ms_; }

void Toast::SetDurationMs(int duration_ms) {
  const int bounded = std::max(duration_ms, 0);
  if (duration_ms_ == bounded) {
    return;
  }
  duration_ms_ = bounded;
  if (!isVisible()) {
    return;
  }
  if (duration_ms_ > 0) {
    dismiss_timer_->start(duration_ms_);
  } else {
    dismiss_timer_->stop();
  }
}

void Toast::Show() {
  dismissed_ = false;
  adjustSize();
  Reposition();
  raise();
  show();
  if (duration_ms_ > 0) {
    dismiss_timer_->start(duration_ms_);
  } else {
    dismiss_timer_->stop();
  }
}

void Toast::Dismiss() {
  dismiss_timer_->stop();
  if (dismissed_) {
    return;
  }
  dismissed_ = true;
  hide();
  emit Dismissed();
}

Toast* Toast::ShowMessage(QWidget* parent, const QString& message,
                          FeedbackLevel level, int duration_ms) {
  auto* toast = new Toast(parent);
  toast->setAttribute(Qt::WA_DeleteOnClose, false);
  toast->SetMessage(message);
  toast->SetLevel(level);
  toast->SetDurationMs(duration_ms);
  QObject::connect(toast, &Toast::Dismissed, toast, &QObject::deleteLater);
  toast->Show();
  return toast;
}

bool Toast::eventFilter(QObject* watched, QEvent* event) {
  if (watched == parentWidget()) {
    if (event->type() == QEvent::Resize || event->type() == QEvent::Move) {
      Reposition();
    } else if (event->type() == QEvent::Hide) {
      Dismiss();
    }
  }
  return QFrame::eventFilter(watched, event);
}

void Toast::changeEvent(QEvent* event) {
  if (event->type() == QEvent::PaletteChange ||
      event->type() == QEvent::StyleChange ||
      event->type() == QEvent::ScreenChangeInternal ||
      event->type() == QEvent::EnabledChange) {
    UpdateIcon();
  }
  QFrame::changeEvent(event);
}

void Toast::showEvent(QShowEvent* event) {
  QFrame::showEvent(event);
  Reposition();
}

void Toast::hideEvent(QHideEvent* event) {
  dismiss_timer_->stop();
  QFrame::hideEvent(event);
}

void Toast::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const QColor accent = SemanticColor(level_, tokens);
  QColor shadow = tokens.shadow_ambient;
  shadow.setAlpha(qRound(shadow.alpha() * 0.65));

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);
  painter.setBrush(shadow);
  painter.drawRoundedRect(QRectF(rect()).adjusted(1.5, 2.5, -1.5, -0.5),
                          tokens.radius_large, tokens.radius_large);

  painter.setPen(QPen(Blend(accent, tokens.border, 42), 1.0));
  painter.setBrush(Blend(accent, tokens.surface_raised, 12));
  painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -2.5),
                          tokens.radius_large, tokens.radius_large);
}

void Toast::ApplyTheme() {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  if (layout() != nullptr) {
    layout()->setContentsMargins(tokens.spacing_lg, tokens.spacing_sm,
                                 tokens.spacing_sm, tokens.spacing_sm + 2);
    layout()->setSpacing(tokens.spacing_sm);
  }
  close_button_->setFixedSize(44, 44);
  ApplyWidgetTheme(this, tone_, true);
  updateGeometry();
  update();
}

void Toast::Reposition() {
  QWidget* host = parentWidget();
  if (host == nullptr) {
    return;
  }
  constexpr int kEdgeMargin = 20;
  const int maximum_width = std::max(160, host->width() - 2 * kEdgeMargin);
  setMaximumWidth(std::min(520, maximum_width));
  adjustSize();
  move(std::max(kEdgeMargin, (host->width() - width()) / 2),
       std::max(kEdgeMargin, host->height() - height() - kEdgeMargin));
}

void Toast::UpdateIcon() {
  close_button_->setIcon(internal::MakePaletteCloseIcon(palette()));
  const QIcon icon = internal::MakePaletteStatusIcon(
      palette(), SemanticColor(level_, GetThemeTokens(this, tone_)),
      IconForLevel(level_));
  icon_label_->setPixmap(
      icon.pixmap(icon_label_->size(), devicePixelRatioF(),
                  isEnabled() ? QIcon::Normal : QIcon::Disabled));
  UpdateAccessibility();
}

void Toast::UpdateAccessibility() {
  const QString content = message_label_->text();
  icon_label_->setAccessibleName(internal::FeedbackLevelAccessibleName(level_));
  setAccessibleName(content);
  setAccessibleDescription(
      internal::FeedbackAccessibleDescription(level_, content));
}

}  // namespace widget
}  // namespace autoviz
