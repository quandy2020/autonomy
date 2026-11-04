/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/loading_overlay.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QEvent>
#include <QHideEvent>
#include <QLabel>
#include <QPaintEvent>
#include <QPainter>
#include <QShowEvent>
#include <QVBoxLayout>

#include "autoviz/common/widget/feedback/progress_ring.hpp"
#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

LoadingOverlay::LoadingOverlay(QWidget* target) : QWidget(nullptr) {
  setObjectName(QStringLiteral("AutovizLoadingOverlay"));
  setAccessibleName(tr("Loading"));
  setFocusPolicy(Qt::NoFocus);
  setAttribute(Qt::WA_StyledBackground, false);
  setAttribute(Qt::WA_TransparentForMouseEvents, false);

  auto* layout = new QVBoxLayout(this);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  layout->setContentsMargins(tokens.spacing_lg, tokens.spacing_lg,
                             tokens.spacing_lg, tokens.spacing_lg);
  layout->setSpacing(tokens.spacing_sm);
  layout->addStretch(1);

  progress_ring_ = new ProgressRing(this);
  progress_ring_->SetIndeterminate(false);
  layout->addWidget(progress_ring_, 0, Qt::AlignHCenter);

  message_label_ = new QLabel(tr("Loading…"), this);
  message_label_->setAlignment(Qt::AlignCenter);
  message_label_->setWordWrap(true);
  layout->addWidget(message_label_);
  layout->addStretch(1);

  hide();
  AttachTo(target);
}

LoadingOverlay::~LoadingOverlay() {
  ReleaseInputFilter();
  if (target_ != nullptr) {
    target_->removeEventFilter(this);
  }
}

void LoadingOverlay::AttachTo(QWidget* target) {
  if (target_ == target) {
    return;
  }
  ReleaseInputFilter();
  hide();
  if (target_ != nullptr) {
    target_->removeEventFilter(this);
  }
  target_ = target;
  if (target_ == nullptr) {
    hide();
    setParent(nullptr);
    return;
  }
  setParent(target_);
  target_->installEventFilter(this);
  SyncGeometryAndVisibility();
}

QString LoadingOverlay::message() const { return message_label_->text(); }

void LoadingOverlay::SetMessage(const QString& message) {
  message_label_->setText(message);
  message_label_->setVisible(!message.isEmpty());
  setAccessibleDescription(message);
}

void LoadingOverlay::SetBlocksInput(bool blocks_input) {
  if (blocks_input_ == blocks_input) {
    return;
  }
  blocks_input_ = blocks_input;
  setAttribute(Qt::WA_TransparentForMouseEvents, !blocks_input_);
  SyncInputFilter();
  if (blocks_input_ && loading_ && isVisible()) {
    setFocus(Qt::OtherFocusReason);
  }
}

void LoadingOverlay::SetTone(style::Tone tone) {
  tone_ = tone;
  progress_ring_->SetTone(tone);
  ApplyWidgetTheme(message_label_, tone, false);
  update();
}

void LoadingOverlay::SetLoading(bool loading) {
  if (loading_ == loading) {
    SyncGeometryAndVisibility();
    return;
  }
  loading_ = loading;
  if (!loading_) {
    SyncInputFilter();
  }
  SyncGeometryAndVisibility();
  emit LoadingChanged(loading_);
}

bool LoadingOverlay::eventFilter(QObject* watched, QEvent* event) {
  if (application_filter_installed_ && blocks_input_ && loading_ &&
      isVisible()) {
    auto* watched_widget = qobject_cast<QWidget*>(watched);
    const bool belongs_to_target =
        watched_widget != nullptr && target_ != nullptr &&
        (watched_widget == target_ || target_->isAncestorOf(watched_widget));
    const bool belongs_to_overlay =
        watched_widget != nullptr &&
        (watched_widget == this || isAncestorOf(watched_widget));
    if (belongs_to_target && !belongs_to_overlay) {
      switch (event->type()) {
        case QEvent::KeyPress:
        case QEvent::KeyRelease:
        case QEvent::ShortcutOverride:
        case QEvent::MouseButtonPress:
        case QEvent::MouseButtonRelease:
        case QEvent::MouseButtonDblClick:
        case QEvent::MouseMove:
        case QEvent::Wheel:
        case QEvent::TouchBegin:
        case QEvent::TouchUpdate:
        case QEvent::TouchEnd:
          return true;
        default:
          break;
      }
    }
  }
  if (watched == target_) {
    switch (event->type()) {
      case QEvent::Resize:
      case QEvent::Move:
      case QEvent::Show:
      case QEvent::ParentChange:
        SyncGeometryAndVisibility();
        break;
      case QEvent::Hide:
        ReleaseInputFilter();
        SyncGeometryAndVisibility();
        break;
      default:
        break;
    }
  }
  return QWidget::eventFilter(watched, event);
}

void LoadingOverlay::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor background = tokens.surface;
  background.setAlpha(224);
  QPainter painter(this);
  painter.fillRect(rect(), background);
}

void LoadingOverlay::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  raise();
  SyncInputFilter();
  if (blocks_input_) {
    setFocus(Qt::OtherFocusReason);
  }
  SyncAnimation();
}

void LoadingOverlay::hideEvent(QHideEvent* event) {
  QWidget::hideEvent(event);
  SyncInputFilter();
  SyncAnimation();
}

void LoadingOverlay::SyncGeometryAndVisibility() {
  if (target_ == nullptr) {
    hide();
    return;
  }
  setGeometry(target_->rect());
  const bool should_show = loading_ && target_->isVisible();
  setVisible(should_show);
  if (should_show) {
    raise();
  }
  SyncAnimation();
}

void LoadingOverlay::SyncAnimation() {
  progress_ring_->SetIndeterminate(loading_ && isVisible());
}

void LoadingOverlay::CapturePreviousFocus() {
  QWidget* focused = QApplication::focusWidget();
  if (focused == nullptr || focused == this || isAncestorOf(focused)) {
    previous_focus_.clear();
    return;
  }
  previous_focus_ = focused;
}

void LoadingOverlay::ReleaseInputFilter() {
  if (application_filter_installed_) {
    if (QCoreApplication* application = QCoreApplication::instance();
        application != nullptr) {
      application->removeEventFilter(this);
    }
    application_filter_installed_ = false;
  }
  RestorePreviousFocus();
  setFocusPolicy(Qt::NoFocus);
  if (hasFocus()) {
    clearFocus();
  }
}

void LoadingOverlay::RestorePreviousFocus() {
  const QPointer<QWidget> previous = previous_focus_;
  previous_focus_.clear();
  if (previous == nullptr || !previous->isVisible() || !previous->isEnabled()) {
    return;
  }
  QWidget* current = QApplication::focusWidget();
  if (current != nullptr && current != this && !isAncestorOf(current)) {
    return;
  }
  previous->setFocus(Qt::OtherFocusReason);
}

void LoadingOverlay::SyncInputFilter() {
  const bool should_install = blocks_input_ && loading_ && isVisible();
  if (!should_install) {
    ReleaseInputFilter();
    return;
  }
  setFocusPolicy(Qt::StrongFocus);
  if (application_filter_installed_) {
    return;
  }
  QCoreApplication* application = QCoreApplication::instance();
  if (application == nullptr) {
    return;
  }
  CapturePreviousFocus();
  application->installEventFilter(this);
  application_filter_installed_ = true;
}

}  // namespace autoviz::widget
