/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/segmented_control.hpp"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QEvent>
#include <QFontMetrics>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPointer>
#include <algorithm>

namespace autoviz {
namespace widget {
namespace {

int HorizontalKeyStep(int key, Qt::LayoutDirection direction) {
  const int visual_step = key == Qt::Key_Left ? -1 : 1;
  return direction == Qt::RightToLeft ? -visual_step : visual_step;
}

QColor WithOpacity(QColor color, qreal opacity) {
  color.setAlpha(qRound(color.alpha() * qBound(0.0, opacity, 1.0)));
  return color;
}

}  // namespace

class SegmentedControl::SegmentButton final : public QAbstractButton {
 public:
  SegmentButton(const QString& text, style::Tone tone, SegmentedControl* parent)
      : QAbstractButton(parent), owner_(parent), tone_(tone) {
    setProperty("autovizRole", QStringLiteral("segment"));
    setText(text);
    setCheckable(true);
    setCursor(Qt::PointingHandCursor);
    setFocusPolicy(Qt::ClickFocus);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    setAccessibleName(text);
  }

  void SetPosition(bool left_edge, bool right_edge) {
    setProperty("autovizLeftEdge", left_edge);
    setProperty("autovizRightEdge", right_edge);
    if (left_edge_ == left_edge && right_edge_ == right_edge) {
      return;
    }
    left_edge_ = left_edge;
    right_edge_ = right_edge;
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
    const ThemeTokens& tokens = GetThemeTokens(this, tone_);
    const QFontMetrics metrics(font());
    return QSize(
        std::max(tokens.control_height_large,
                 metrics.horizontalAdvance(text()) + 2 * tokens.spacing_md),
        std::max(tokens.control_height_small,
                 metrics.height() + tokens.spacing_sm));
  }

 protected:
  void keyPressEvent(QKeyEvent* event) override {
    if (owner_->HandleNavigationKey(this, event)) {
      return;
    }
    QAbstractButton::keyPressEvent(event);
  }

  void paintEvent(QPaintEvent* event) override {
    Q_UNUSED(event);
    const ThemeTokens& tokens = GetThemeTokens(this, tone_);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QColor fill = Qt::transparent;
    QColor text_color = tokens.text_secondary;
    if (!isEnabled()) {
      text_color = tokens.disabled;
      if (isChecked()) {
        fill = WithOpacity(tokens.surface_selected, 0.7);
        text_color = WithOpacity(tokens.accent_text, 0.55);
      }
    } else if (isDown()) {
      fill = isChecked() ? tokens.surface_pressed : tokens.surface_material;
      text_color = isChecked() ? tokens.accent_text : tokens.text;
    } else if (isChecked()) {
      // CapCut: soft gray plate + olive ink, no border.
      fill = tokens.surface_selected;
      text_color = tokens.accent_text;
    } else if (underMouse()) {
      fill = tokens.surface_hovered;
      text_color = tokens.text;
    }

    const QRectF bounds = QRectF(rect()).adjusted(1.0, 1.0, -1.0, -1.0);
    const qreal radius =
        qMin<qreal>(tokens.radius_small, bounds.height() * 0.5);
    painter.setPen(Qt::NoPen);
    painter.setBrush(fill);
    painter.drawRoundedRect(bounds, radius, radius);

    QFont label_font = font();
    if (isChecked()) {
      label_font.setWeight(QFont::DemiBold);
    }
    painter.setFont(label_font);
    painter.setPen(text_color);
    painter.drawText(rect().adjusted(8, 0, -8, 0), Qt::AlignCenter, text());
  }

 private:
  SegmentedControl* owner_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  bool left_edge_ = false;
  bool right_edge_ = false;
};

SegmentedControl::SegmentedControl(QWidget* parent)
    : QWidget(parent),
      button_group_(new QButtonGroup(this)),
      layout_(new QHBoxLayout(this)) {
  setObjectName(QStringLiteral("AutovizSegmentedControl"));
  setProperty("autovizRole", QStringLiteral("segmentedControl"));
  setAttribute(Qt::WA_StyledBackground, false);
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  layout_->setContentsMargins(2, 2, 2, 2);
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
  } else {
    UpdateFocusPolicies();
    if (current_index_changed) {
      emit CurrentIndexChanged(current_index_);
    }
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
  UpdateFocusPolicies();

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
  UpdateFocusPolicies();
  const QVariant data = current_data();
  QPointer<SegmentedControl> guard(this);
  emit CurrentIndexChanged(index);
  if (guard != nullptr) {
    emit CurrentDataChanged(data);
  }
}

void SegmentedControl::SetSegmentEnabled(int index, bool enabled) {
  if (index < 0 || index >= static_cast<int>(buttons_.size())) {
    return;
  }
  const bool restore_focus = !enabled && buttons_[index]->hasFocus();
  buttons_[index]->setEnabled(enabled);
  UpdateFocusPolicies();
  if (restore_focus) {
    const int focus_index = is_segment_enabled(current_index_)
                                ? current_index_
                                : FirstEnabledIndex();
    if (focus_index >= 0) {
      buttons_[focus_index]->setFocus(Qt::OtherFocusReason);
    }
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

bool SegmentedControl::HandleNavigationKey(SegmentButton* button,
                                           QKeyEvent* event) {
  const int source_index = static_cast<int>(buttons_.indexOf(button));
  if (source_index < 0) {
    return false;
  }

  int target_index = -1;
  switch (event->key()) {
    case Qt::Key_Left:
    case Qt::Key_Right:
      target_index = NextEnabledIndex(
          source_index, HorizontalKeyStep(event->key(), layoutDirection()));
      break;
    case Qt::Key_Home:
      target_index = FirstEnabledIndex();
      break;
    case Qt::Key_End:
      target_index = LastEnabledIndex();
      break;
    default:
      return false;
  }

  event->accept();
  if (target_index < 0) {
    return true;
  }

  QPointer<SegmentedControl> guard(this);
  SetCurrentIndex(target_index);
  if (guard != nullptr) {
    const int focus_index = is_segment_enabled(current_index_)
                                ? current_index_
                                : FirstEnabledIndex();
    if (focus_index >= 0) {
      buttons_[focus_index]->setFocus(Qt::OtherFocusReason);
    }
  }
  return true;
}

int SegmentedControl::FirstEnabledIndex() const {
  for (int index = 0; index < static_cast<int>(buttons_.size()); ++index) {
    if (buttons_[index]->isEnabled()) {
      return index;
    }
  }
  return -1;
}

int SegmentedControl::LastEnabledIndex() const {
  for (int index = static_cast<int>(buttons_.size()) - 1; index >= 0; --index) {
    if (buttons_[index]->isEnabled()) {
      return index;
    }
  }
  return -1;
}

int SegmentedControl::NextEnabledIndex(int index, int step) const {
  if (step == 0) {
    return -1;
  }
  for (int candidate = index + step;
       candidate >= 0 && candidate < static_cast<int>(buttons_.size());
       candidate += step) {
    if (buttons_[candidate]->isEnabled()) {
      return candidate;
    }
  }
  return -1;
}

void SegmentedControl::UpdateButtonPositions() {
  const int button_count = static_cast<int>(buttons_.size());
  const bool right_to_left = layoutDirection() == Qt::RightToLeft;
  for (int index = 0; index < button_count; ++index) {
    const bool left_edge = index == (right_to_left ? button_count - 1 : 0);
    const bool right_edge = index == (right_to_left ? 0 : button_count - 1);
    buttons_[index]->SetPosition(left_edge, right_edge);
  }
}

void SegmentedControl::UpdateFocusPolicies() {
  const int focus_index =
      is_segment_enabled(current_index_) ? current_index_ : FirstEnabledIndex();
  for (int index = 0; index < static_cast<int>(buttons_.size()); ++index) {
    buttons_[index]->setFocusPolicy(index == focus_index ? Qt::StrongFocus
                                                         : Qt::ClickFocus);
  }
}

void SegmentedControl::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const QRectF bounds = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
  const qreal radius = qMin<qreal>(tokens.radius_medium, bounds.height() * 0.5);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(tokens.border_subtle, 1.0));
  painter.setBrush(tokens.surface_sunken);
  painter.drawRoundedRect(bounds, radius, radius);
}

void SegmentedControl::changeEvent(QEvent* event) {
  QWidget::changeEvent(event);
  if (event->type() == QEvent::LayoutDirectionChange) {
    layout_->invalidate();
    UpdateButtonPositions();
  }
}

}  // namespace widget
}  // namespace autoviz
