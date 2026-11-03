/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/navigation_rail.hpp"

#include <QButtonGroup>
#include <QKeyEvent>
#include <QPointer>
#include <QToolButton>
#include <QVBoxLayout>
#include <utility>

namespace autoviz {
namespace widget {

NavigationRail::NavigationRail(QWidget* parent) : QFrame(parent) {
  setObjectName(QStringLiteral("AutovizNavigationRail"));
  setFrameShape(QFrame::NoFrame);
  setFocusPolicy(Qt::StrongFocus);
  setAccessibleName(tr("Main navigation"));
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);

  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(4, 4, 4, 4);
  layout_->setSpacing(4);
  layout_->addStretch(1);

  button_group_ = new QButtonGroup(this);
  button_group_->setExclusive(true);
}

const QVector<NavigationItem>& NavigationRail::items() const { return items_; }

void NavigationRail::SetItems(QVector<NavigationItem> items) {
  items_ = std::move(items);
  if (current_index_ >= static_cast<int>(items_.size()) ||
      !IsSelectable(current_index_)) {
    current_index_ = -1;
    for (int i = 0; i < static_cast<int>(items_.size()); ++i) {
      if (IsSelectable(i)) {
        current_index_ = i;
        break;
      }
    }
  }
  Rebuild();
  if (current_index_ >= 0) {
    emit CurrentChanged(current_index_, items_[current_index_].data);
  }
}

void NavigationRail::AddItem(NavigationItem item) {
  items_.push_back(std::move(item));
  if (current_index_ < 0 && IsSelectable(static_cast<int>(items_.size()) - 1)) {
    current_index_ = static_cast<int>(items_.size()) - 1;
  }
  Rebuild();
}

void NavigationRail::Clear() {
  items_.clear();
  current_index_ = -1;
  Rebuild();
}

int NavigationRail::current_index() const { return current_index_; }

void NavigationRail::SetCurrentIndex(int index) {
  if (!IsSelectable(index) || current_index_ == index) {
    return;
  }
  current_index_ = index;
  for (int i = 0; i < static_cast<int>(buttons_.size()); ++i) {
    buttons_[i]->setChecked(i == index);
  }
  setFocus(Qt::OtherFocusReason);
  emit CurrentChanged(index, items_[index].data);
}

bool NavigationRail::labels_visible() const { return labels_visible_; }

void NavigationRail::SetLabelsVisible(bool visible) {
  if (labels_visible_ == visible) {
    return;
  }
  labels_visible_ = visible;
  for (int i = 0; i < static_cast<int>(buttons_.size()); ++i) {
    buttons_[i]->setText(visible ? items_[i].text : QString());
    buttons_[i]->setToolTip(visible ? QString() : items_[i].text);
    buttons_[i]->setToolButtonStyle(visible ? Qt::ToolButtonTextBesideIcon
                                            : Qt::ToolButtonIconOnly);
  }
  updateGeometry();
}

void NavigationRail::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Up:
      MoveCurrent(-1);
      event->accept();
      return;
    case Qt::Key_Down:
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
  QFrame::keyPressEvent(event);
}

void NavigationRail::Rebuild() {
  while (QLayoutItem* child = layout_->takeAt(0)) {
    if (child->widget() != nullptr) {
      button_group_->removeButton(
          qobject_cast<QAbstractButton*>(child->widget()));
      child->widget()->deleteLater();
    }
    delete child;
  }
  buttons_.clear();

  for (int i = 0; i < static_cast<int>(items_.size()); ++i) {
    auto* button = new QToolButton(this);
    button->setObjectName(QStringLiteral("AutovizNavigationItem"));
    button->setCheckable(true);
    button->setChecked(i == current_index_);
    button->setEnabled(items_[i].enabled);
    button->setIcon(items_[i].icon);
    button->setText(labels_visible_ ? items_[i].text : QString());
    button->setToolTip(labels_visible_ ? QString() : items_[i].text);
    button->setToolButtonStyle(labels_visible_ ? Qt::ToolButtonTextBesideIcon
                                               : Qt::ToolButtonIconOnly);
    button->setFocusPolicy(Qt::NoFocus);
    button->setAccessibleName(items_[i].text);
    button->setMinimumHeight(40);
    button_group_->addButton(button, i);
    connect(button, &QToolButton::clicked, this, [this, i]() {
      setFocus(Qt::MouseFocusReason);
      const QVariant data = items_[i].data;
      QPointer<NavigationRail> guard(this);
      SetCurrentIndex(i);
      if (guard != nullptr) {
        emit ItemActivated(i, data);
      }
    });
    buttons_.push_back(button);
    layout_->addWidget(button);
  }
  layout_->addStretch(1);
  updateGeometry();
}

void NavigationRail::MoveCurrent(int step) {
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

void NavigationRail::ActivateCurrent() {
  if (IsSelectable(current_index_)) {
    emit ItemActivated(current_index_, items_[current_index_].data);
  }
}

bool NavigationRail::IsSelectable(int index) const {
  return index >= 0 && index < static_cast<int>(items_.size()) &&
         items_[index].enabled;
}

}  // namespace widget
}  // namespace autoviz
