/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/segmented_control.hpp"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QFontMetrics>
#include <QHBoxLayout>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPointer>
#include <algorithm>

namespace autoviz {
namespace widget {

class SegmentedControl::SegmentButton final : public QAbstractButton {
 public:
  SegmentButton(const QString& text, style::Tone tone, QWidget* parent)
      : QAbstractButton(parent), tone_(tone) {
    setProperty("autovizRole", QStringLiteral("segment"));
    setText(text);
    setCheckable(true);
    setCursor(Qt::PointingHandCursor);
    setFocusPolicy(Qt::StrongFocus);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    setAccessibleName(text);
  }

  void SetPosition(bool first, bool last) {
    first_ = first;
    last_ = last;
    update();
  }

  void SetTone(style::Tone tone) {
    if (tone_ == tone) {
      return;
    }
    tone_ = tone;
    updateGeometry();
    update();
  }

  QSize sizeHint() const override {
    const ThemeTokens& tokens = GetThemeTokens(tone_);
    const QFontMetrics metrics(font());
    return QSize(
        std::max(tokens.control_height_large,
                 metrics.horizontalAdvance(text()) + 2 * tokens.spacing_md),
        std::max(tokens.control_height_small,
                 metrics.height() + tokens.spacing_sm));
  }

 protected:
  void paintEvent(QPaintEvent* event) override {
    Q_UNUSED(event);
    const ThemeTokens& tokens = GetThemeTokens(tone_);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QColor fill = tokens.surface_raised;
    QColor text_color = tokens.text;
    QColor border = tokens.border;
    if (!isEnabled()) {
      fill = tokens.surface;
      text_color = tokens.disabled;
      border = tokens.disabled;
    } else if (isDown()) {
      fill = tokens.accent_pressed;
      text_color = tokens.on_accent;
      border = tokens.accent_pressed;
    } else if (isChecked()) {
      fill = tokens.accent;
      text_color = tokens.on_accent;
      border = tokens.accent;
    } else if (underMouse()) {
      fill = tokens.surface_hovered;
    }

    const qreal stroke_width =
        hasFocus() ? static_cast<qreal>(tokens.focus_ring_width) : 1.0;
    const qreal inset = stroke_width * 0.5;
    const QRectF bounds = QRectF(rect()).adjusted(inset, inset, -inset, -inset);
    const qreal radius = tokens.radius_medium;
    QPainterPath path;
    if (first_ && last_) {
      path.addRoundedRect(bounds, radius, radius);
    } else {
      const qreal left_radius = first_ ? radius : 0.0;
      const qreal right_radius = last_ ? radius : 0.0;
      path.moveTo(bounds.left() + left_radius, bounds.top());
      path.lineTo(bounds.right() - right_radius, bounds.top());
      if (right_radius > 0.0) {
        path.quadTo(bounds.right(), bounds.top(), bounds.right(),
                    bounds.top() + right_radius);
      }
      path.lineTo(bounds.right(), bounds.bottom() - right_radius);
      if (right_radius > 0.0) {
        path.quadTo(bounds.right(), bounds.bottom(),
                    bounds.right() - right_radius, bounds.bottom());
      }
      path.lineTo(bounds.left() + left_radius, bounds.bottom());
      if (left_radius > 0.0) {
        path.quadTo(bounds.left(), bounds.bottom(), bounds.left(),
                    bounds.bottom() - left_radius);
      }
      path.lineTo(bounds.left(), bounds.top() + left_radius);
      if (left_radius > 0.0) {
        path.quadTo(bounds.left(), bounds.top(), bounds.left() + left_radius,
                    bounds.top());
      }
      path.closeSubpath();
    }

    painter.setPen(QPen(hasFocus() ? tokens.focus_ring : border, stroke_width));
    painter.setBrush(fill);
    painter.drawPath(path);
    painter.setPen(text_color);
    painter.drawText(rect().adjusted(8, 0, -8, 0), Qt::AlignCenter, text());
  }

 private:
  style::Tone tone_ = style::Tone::Shell;
  bool first_ = false;
  bool last_ = false;
};

SegmentedControl::SegmentedControl(QWidget* parent)
    : QWidget(parent),
      button_group_(new QButtonGroup(this)),
      layout_(new QHBoxLayout(this)) {
  setObjectName(QStringLiteral("AutovizSegmentedControl"));
  setProperty("autovizRole", QStringLiteral("segmentedControl"));
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  layout_->setContentsMargins(0, 0, 0, 0);
  layout_->setSpacing(0);
  button_group_->setExclusive(true);
}

SegmentedControl::~SegmentedControl() = default;

int SegmentedControl::AddSegment(const QString& text, const QVariant& data) {
  return InsertSegment(static_cast<int>(buttons_.size()), text, data);
}

int SegmentedControl::InsertSegment(int index, const QString& text,
                                    const QVariant& data) {
  index = std::clamp(index, 0, static_cast<int>(buttons_.size()));
  auto* button = new SegmentButton(text, tone_, this);
  connect(button, &QAbstractButton::clicked, this,
          [this, button]() { HandleButtonClicked(button); });

  buttons_.insert(index, button);
  data_.insert(index, data);
  button_group_->addButton(button);
  layout_->insertWidget(index, button);
  const bool current_index_changed = current_index_ >= index;
  if (current_index_changed) {
    ++current_index_;
  }
  UpdateButtonPositions();

  if (current_index_ < 0) {
    SetCurrentIndex(0);
  } else if (current_index_changed) {
    emit CurrentIndexChanged(current_index_);
  }
  return index;
}

void SegmentedControl::RemoveSegment(int index) {
  if (index < 0 || index >= static_cast<int>(buttons_.size())) {
    return;
  }

  const bool removed_current = index == current_index_;
  const bool current_index_shifted = current_index_ > index;
  SegmentButton* button = buttons_.takeAt(index);
  data_.removeAt(index);
  button_group_->removeButton(button);
  layout_->removeWidget(button);
  button->hide();
  button->deleteLater();

  if (current_index_ > index) {
    --current_index_;
  } else if (removed_current) {
    current_index_ = -1;
  }
  UpdateButtonPositions();

  if (removed_current && !buttons_.isEmpty()) {
    SetCurrentIndex(std::min(index, static_cast<int>(buttons_.size()) - 1));
  } else if (removed_current) {
    QPointer<SegmentedControl> guard(this);
    emit CurrentIndexChanged(-1);
    if (guard != nullptr) {
      emit CurrentDataChanged(QVariant());
    }
  } else if (current_index_shifted) {
    emit CurrentIndexChanged(current_index_);
  }
}

void SegmentedControl::Clear() {
  if (buttons_.isEmpty()) {
    return;
  }
  for (SegmentButton* button : buttons_) {
    button_group_->removeButton(button);
    layout_->removeWidget(button);
    button->hide();
    button->deleteLater();
  }
  buttons_.clear();
  data_.clear();
  current_index_ = -1;
  QPointer<SegmentedControl> guard(this);
  emit CurrentIndexChanged(-1);
  if (guard != nullptr) {
    emit CurrentDataChanged(QVariant());
  }
}

int SegmentedControl::count() const {
  return static_cast<int>(buttons_.size());
}

int SegmentedControl::current_index() const { return current_index_; }

QVariant SegmentedControl::current_data() const {
  return segment_data(current_index_);
}

QString SegmentedControl::segment_text(int index) const {
  if (index < 0 || index >= static_cast<int>(buttons_.size())) {
    return {};
  }
  return buttons_[index]->text();
}

QVariant SegmentedControl::segment_data(int index) const {
  return data_.value(index);
}

bool SegmentedControl::is_segment_enabled(int index) const {
  return index >= 0 && index < static_cast<int>(buttons_.size()) &&
         buttons_[index]->isEnabled();
}

void SegmentedControl::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  for (SegmentButton* button : buttons_) {
    button->SetTone(tone);
  }
  updateGeometry();
  update();
}

void SegmentedControl::SetCurrentIndex(int index) {
  if (index < -1 || index >= static_cast<int>(buttons_.size()) ||
      index == current_index_) {
    return;
  }

  if (index < 0) {
    button_group_->setExclusive(false);
    for (SegmentButton* button : buttons_) {
      button->setChecked(false);
    }
    button_group_->setExclusive(true);
  } else {
    buttons_[index]->setChecked(true);
  }
  current_index_ = index;
  const QVariant data = current_data();
  QPointer<SegmentedControl> guard(this);
  emit CurrentIndexChanged(index);
  if (guard != nullptr) {
    emit CurrentDataChanged(data);
  }
}

void SegmentedControl::SetSegmentEnabled(int index, bool enabled) {
  if (index >= 0 && index < static_cast<int>(buttons_.size())) {
    buttons_[index]->setEnabled(enabled);
  }
}

void SegmentedControl::SetSegmentText(int index, const QString& text) {
  if (index < 0 || index >= static_cast<int>(buttons_.size())) {
    return;
  }
  buttons_[index]->setText(text);
  buttons_[index]->setAccessibleName(text);
  updateGeometry();
}

void SegmentedControl::HandleButtonClicked(SegmentButton* button) {
  const int index = static_cast<int>(buttons_.indexOf(button));
  if (index < 0) {
    return;
  }
  QPointer<SegmentedControl> guard(this);
  SetCurrentIndex(index);
  if (guard.isNull()) {
    return;
  }
  emit SegmentClicked(index);
}

void SegmentedControl::UpdateButtonPositions() {
  const int button_count = static_cast<int>(buttons_.size());
  for (int index = 0; index < button_count; ++index) {
    buttons_[index]->SetPosition(index == 0, index == button_count - 1);
  }
}

}  // namespace widget
}  // namespace autoviz
