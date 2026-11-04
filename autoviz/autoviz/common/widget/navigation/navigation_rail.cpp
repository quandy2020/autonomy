/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/navigation_rail.hpp"

#include <QButtonGroup>
#include <QFocusEvent>
#include <QKeyEvent>
#include <QPainter>
#include <QPointer>
#include <QStyleOptionToolButton>
#include <QToolButton>
#include <QVBoxLayout>
#include <utility>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz {
namespace widget {
namespace {

QColor Blend(const QColor& foreground, const QColor& background, int alpha) {
  const int bounded = qBound(0, alpha, 255);
  const int inverse = 255 - bounded;
  return QColor(
      (foreground.red() * bounded + background.red() * inverse) / 255,
      (foreground.green() * bounded + background.green() * inverse) / 255,
      (foreground.blue() * bounded + background.blue() * inverse) / 255,
      (foreground.alpha() * bounded + background.alpha() * inverse) / 255);
}

class NavigationItemButton final : public QToolButton {
 public:
  explicit NavigationItemButton(style::Tone tone, QWidget* parent)
      : QToolButton(parent), tone_(tone) {
    setAttribute(Qt::WA_Hover);
    setAutoRaise(true);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  }

  void SetTone(style::Tone tone) {
    tone_ = tone;
    update();
  }

 protected:
  void paintEvent(QPaintEvent* event) override {
    Q_UNUSED(event);
    const ThemeTokens& tokens = GetThemeTokens(this, tone_);
    const bool selected = isChecked();
    const bool pressed = isDown();
    const bool hovered = underMouse();
    const bool keyboard_focus =
        selected && parentWidget() != nullptr && parentWidget()->hasFocus();

    QColor fill = Qt::transparent;
    if (selected) {
      fill = Blend(tokens.accent, tokens.surface, pressed ? 54 : 34);
    } else if (pressed) {
      fill = Blend(tokens.text, tokens.surface, 28);
    } else if (hovered) {
      fill = tokens.surface_hovered;
    }

    const QRectF bounds = QRectF(rect()).adjusted(1.0, 1.0, -1.0, -1.0);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBrush(fill);
    if (keyboard_focus) {
      painter.setPen(QPen(tokens.focus_ring, tokens.focus_ring_width));
    } else if (selected) {
      painter.setPen(QPen(Blend(tokens.accent, tokens.border, 92), 1.0));
    } else {
      painter.setPen(Qt::NoPen);
    }
    painter.drawRoundedRect(bounds, tokens.radius_medium, tokens.radius_medium);

    // Keep compact desktop labels at body-text contrast. Selection already has
    // an accent fill and outline, so accent-colored text would communicate the
    // same state twice while falling below 4.5:1 on the tinted surface.
    QColor foreground = tokens.text;
    if (!isEnabled()) {
      foreground = tokens.disabled;
    }
    QStyleOptionToolButton option;
    initStyleOption(&option);
    option.rect = rect().adjusted(tokens.spacing_sm, 0, -tokens.spacing_sm, 0);
    option.palette.setColor(QPalette::ButtonText, foreground);
    option.palette.setColor(QPalette::Text, foreground);
    option.state &= ~QStyle::State_Sunken;
    option.state &= ~QStyle::State_Raised;
    style()->drawControl(QStyle::CE_ToolButtonLabel, &option, &painter, this);
  }

 private:
  style::Tone tone_;
};

}  // namespace

NavigationRail::NavigationRail(QWidget* parent) : QFrame(parent) {
  setObjectName(QStringLiteral("AutovizNavigationRail"));
  setProperty("autovizRole", QStringLiteral("navigationRail"));
  setFrameShape(QFrame::NoFrame);
  setAttribute(Qt::WA_TranslucentBackground);
  setFocusPolicy(Qt::StrongFocus);
  setAccessibleName(tr("Main navigation"));
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);

  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(4, 4, 4, 4);
  layout_->setSpacing(4);
  layout_->addStretch(1);

  button_group_ = new QButtonGroup(this);
  button_group_->setExclusive(true);
  ApplyTheme();
  UpdateAccessibility();
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
  UpdateAccessibility();
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

void NavigationRail::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyTheme();
}

void NavigationRail::focusInEvent(QFocusEvent* event) {
  QFrame::focusInEvent(event);
  for (QToolButton* button : buttons_) {
    button->update();
  }
}

void NavigationRail::focusOutEvent(QFocusEvent* event) {
  QFrame::focusOutEvent(event);
  for (QToolButton* button : buttons_) {
    button->update();
  }
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
    auto* button = new NavigationItemButton(tone_, this);
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
    button->setProperty("autovizRole", QStringLiteral("navigationItem"));
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
  ApplyTheme();
  UpdateAccessibility();
  updateGeometry();
}

void NavigationRail::ApplyTheme() {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  layout_->setContentsMargins(tokens.spacing_xs, tokens.spacing_xs,
                              tokens.spacing_xs, tokens.spacing_xs);
  layout_->setSpacing(tokens.spacing_xs);
  for (QToolButton* button : buttons_) {
    static_cast<NavigationItemButton*>(button)->SetTone(tone_);
    button->setMinimumHeight(tokens.control_height_large);
  }
  ApplyWidgetTheme(this, tone_, true);
  updateGeometry();
  update();
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

void NavigationRail::UpdateAccessibility() {
  if (IsSelectable(current_index_)) {
    setAccessibleDescription(tr("Current item: %1, %2 of %3")
                                 .arg(items_[current_index_].text)
                                 .arg(current_index_ + 1)
                                 .arg(items_.size()));
    return;
  }
  setAccessibleDescription(items_.isEmpty() ? tr("No navigation items")
                                            : tr("No current item"));
}

}  // namespace widget
}  // namespace autoviz
