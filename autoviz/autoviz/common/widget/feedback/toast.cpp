/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/feedback/toast.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QLabel>
#include <QPushButton>
#include <QShowEvent>
#include <QStyle>
#include <QTimer>
#include <algorithm>

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

}  // namespace

Toast::Toast(QWidget* parent) : QFrame(parent) {
  setObjectName(QStringLiteral("AutovizToast"));
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setAutoFillBackground(true);
  setFocusPolicy(Qt::NoFocus);
  setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);

  auto* layout = new QHBoxLayout(this);
  layout->setContentsMargins(12, 8, 8, 8);
  layout->setSpacing(8);

  icon_label_ = new QLabel(this);
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
  close_button_->setFlat(true);
  close_button_->setIcon(style()->standardIcon(QStyle::SP_DialogCloseButton));
  close_button_->setFixedSize(26, 26);
  close_button_->setAccessibleName(tr("Dismiss notification"));
  connect(close_button_, &QPushButton::clicked, this, &Toast::Dismiss);
  layout->addWidget(close_button_);

  dismiss_timer_ = new QTimer(this);
  dismiss_timer_->setSingleShot(true);
  connect(dismiss_timer_, &QTimer::timeout, this, &Toast::Dismiss);

  if (parent != nullptr) {
    parent->installEventFilter(this);
  }
  UpdateIcon();
}

QString Toast::message() const { return message_label_->text(); }

void Toast::SetMessage(const QString& message) {
  message_label_->setText(message);
  setAccessibleName(message);
  setAccessibleDescription(message);
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
      event->type() == QEvent::ScreenChangeInternal) {
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
  const QIcon icon = style()->standardIcon(IconForLevel(level_));
  icon_label_->setPixmap(
      icon.pixmap(icon_label_->size(), devicePixelRatioF(),
                  isEnabled() ? QIcon::Normal : QIcon::Disabled));
}

}  // namespace widget
}  // namespace autoviz
