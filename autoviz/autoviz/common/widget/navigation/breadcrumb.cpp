/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/breadcrumb.hpp"

#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPainter>
#include <QPointer>
#include <QToolButton>
#include <utility>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz {
namespace widget {
namespace {

class BreadcrumbSeparator final : public QWidget {
 public:
  explicit BreadcrumbSeparator(QWidget* parent) : QWidget(parent) {
    setObjectName(QStringLiteral("AutovizBreadcrumbSeparator"));
    setProperty("autovizRole", QStringLiteral("decorative"));
    setFocusPolicy(Qt::NoFocus);
    setAttribute(Qt::WA_TransparentForMouseEvents);
  }

  QSize sizeHint() const override {
    return QSize(fontMetrics().horizontalAdvance(QStringLiteral("›")) + 4,
                 fontMetrics().height());
  }

 protected:
  void paintEvent(QPaintEvent* event) override {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setPen(palette().color(QPalette::Text));
    painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("›"));
  }
};

}  // namespace

Breadcrumb::Breadcrumb(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizBreadcrumb"));
  setProperty("autovizRole", QStringLiteral("breadcrumb"));
  setFocusPolicy(Qt::StrongFocus);
  setAccessibleName(tr("Breadcrumb navigation"));

  layout_ = new QHBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);
  layout_->setSpacing(4);
  layout_->addStretch(1);
  ApplyTheme();
  UpdateAccessibility();
}

const QVector<BreadcrumbItem>& Breadcrumb::items() const { return items_; }

void Breadcrumb::SetItems(QVector<BreadcrumbItem> items) {
  items_ = std::move(items);
  current_index_ = items_.isEmpty() ? -1 : static_cast<int>(items_.size()) - 1;
  Rebuild();
  emit CurrentIndexChanged(current_index_);
}

void Breadcrumb::AppendItem(BreadcrumbItem item) {
  items_.push_back(std::move(item));
  current_index_ = static_cast<int>(items_.size()) - 1;
  Rebuild();
  emit CurrentIndexChanged(current_index_);
}

void Breadcrumb::Clear() {
  if (items_.isEmpty()) {
    return;
  }
  items_.clear();
  current_index_ = -1;
  Rebuild();
  emit CurrentIndexChanged(-1);
}

int Breadcrumb::current_index() const { return current_index_; }

void Breadcrumb::SetCurrentIndex(int index) {
  if (!IsSelectable(index) || current_index_ == index) {
    return;
  }
  current_index_ = index;
  for (int i = 0; i < static_cast<int>(buttons_.size()); ++i) {
    buttons_[i]->setChecked(i == current_index_);
  }
  setFocus(Qt::OtherFocusReason);
  UpdateAccessibility();
  emit CurrentIndexChanged(index);
}

void Breadcrumb::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyTheme();
}

void Breadcrumb::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Left:
      MoveCurrent(-1);
      event->accept();
      return;
    case Qt::Key_Right:
      MoveCurrent(1);
      event->accept();
      return;
    case Qt::Key_Home:
      for (int i = 0; i < static_cast<int>(items_.size()); ++i) {
        if (IsSelectable(i)) {
          SetCurrentIndex(i);
          break;
        }
      }
      event->accept();
      return;
    case Qt::Key_End:
      for (int i = static_cast<int>(items_.size()) - 1; i >= 0; --i) {
        if (IsSelectable(i)) {
          SetCurrentIndex(i);
          break;
        }
      }
      event->accept();
      return;
    case Qt::Key_Return:
    case Qt::Key_Enter:
    case Qt::Key_Space:
      ActivateCurrent();
      event->accept();
      return;
    default:
      break;
  }
  QWidget::keyPressEvent(event);
}

void Breadcrumb::Rebuild() {
  while (QLayoutItem* child = layout_->takeAt(0)) {
    if (child->widget() != nullptr) {
      if (auto* button = qobject_cast<QToolButton*>(child->widget());
          button != nullptr) {
        button->setAutoExclusive(false);
        button->setChecked(false);
      }
      child->widget()->hide();
      child->widget()->deleteLater();
    }
    delete child;
  }
  buttons_.clear();

  for (int i = 0; i < static_cast<int>(items_.size()); ++i) {
    if (i > 0) {
      auto* separator = new BreadcrumbSeparator(this);
      layout_->addWidget(separator);
    }

    auto* button = new QToolButton(this);
    button->setObjectName(QStringLiteral("AutovizBreadcrumbItem"));
    button->setText(items_[i].text);
    button->setAutoRaise(true);
    button->setCheckable(true);
    button->setAutoExclusive(true);
    button->setChecked(i == current_index_);
    button->setEnabled(items_[i].enabled);
    button->setFocusPolicy(Qt::NoFocus);
    button->setAccessibleName(items_[i].text);
    button->setProperty("autovizRole", QStringLiteral("breadcrumbItem"));
    connect(button, &QToolButton::clicked, this, [this, i]() {
      setFocus(Qt::MouseFocusReason);
      const QVariant data = items_[i].data;
      QPointer<Breadcrumb> guard(this);
      SetCurrentIndex(i);
      if (guard != nullptr) {
        emit ItemActivated(i, data);
      }
    });
    buttons_.push_back(button);
    layout_->addWidget(button);
  }
  layout_->addStretch(1);
  ApplyTheme();
  UpdateAccessibility();
  updateGeometry();
}

void Breadcrumb::ApplyTheme() {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  layout_->setSpacing(tokens.spacing_xs);
  for (QToolButton* button : buttons_) {
    button->setMinimumHeight(tokens.control_height_small);
  }
  ApplyWidgetTheme(this, tone_, true);
  updateGeometry();
  update();
}

void Breadcrumb::MoveCurrent(int step) {
  if (items_.isEmpty()) {
    return;
  }
  int index = current_index_;
  if (index < 0) {
    index = step > 0 ? -1 : static_cast<int>(items_.size());
  }
  for (int candidate = index + step;
       candidate >= 0 && candidate < static_cast<int>(items_.size());
       candidate += step) {
    if (IsSelectable(candidate)) {
      SetCurrentIndex(candidate);
      return;
    }
  }
}

void Breadcrumb::ActivateCurrent() {
  if (!IsSelectable(current_index_)) {
    return;
  }
  emit ItemActivated(current_index_, items_[current_index_].data);
}

bool Breadcrumb::IsSelectable(int index) const {
  return index >= 0 && index < static_cast<int>(items_.size()) &&
         items_[index].enabled;
}

void Breadcrumb::UpdateAccessibility() {
  if (IsSelectable(current_index_)) {
    setAccessibleDescription(tr("Current item: %1, %2 of %3")
                                 .arg(items_[current_index_].text)
                                 .arg(current_index_ + 1)
                                 .arg(items_.size()));
    return;
  }
  setAccessibleDescription(items_.isEmpty() ? tr("No breadcrumb items")
                                            : tr("No current item"));
}

}  // namespace widget
}  // namespace autoviz
