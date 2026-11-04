/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/filter_bar.hpp"

#include <QHBoxLayout>
#include <QHideEvent>
#include <QPointer>
#include <QSignalBlocker>
#include <QTimer>
#include <QWidget>
#include <algorithm>

#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/button.hpp"
#include "autoviz/common/widget/input/search_box.hpp"

namespace autoviz::widget {

FilterBar::FilterBar(QWidget* parent)
    : QFrame(parent), debounce_timer_(new QTimer(this)) {
  setObjectName(QStringLiteral("AutovizFilterBar"));
  setFrameStyle(QFrame::NoFrame);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setAccessibleName(tr("Filters"));

  auto* root = new QHBoxLayout(this);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(tokens.spacing_sm);

  search_box_ = new SearchBox(this);
  search_box_->SetSearchBoxSize(SearchBox::Size::kSmall);
  search_box_->setAccessibleName(tr("Filter results"));
  search_box_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  root->addWidget(search_box_, 2);

  filters_layout_ = new QHBoxLayout;
  filters_layout_->setContentsMargins(0, 0, 0, 0);
  filters_layout_->setSpacing(tokens.spacing_sm);
  root->addLayout(filters_layout_, 3);

  clear_button_ = new Button(tr("Clear"), this);
  clear_button_->SetVariant(Button::Variant::kSubtle);
  clear_button_->SetButtonSize(Button::Size::kSmall);
  clear_button_->setAccessibleName(tr("Clear filters"));
  root->addWidget(clear_button_);

  debounce_timer_->setSingleShot(true);
  debounce_timer_->setTimerType(Qt::CoarseTimer);
  connect(debounce_timer_, &QTimer::timeout, this,
          [this]() { emit QueryChanged(query()); });
  connect(search_box_, &QLineEdit::textChanged, this, [this]() {
    UpdateClearButton();
    ScheduleQueryChanged();
  });
  connect(search_box_, &SearchBox::SearchRequested, this,
          [this](const QString&) { Submit(); });
  connect(clear_button_, &QAbstractButton::clicked, this, &FilterBar::Clear);

  SetTone(tone_);
  UpdateClearButton();
}

QString FilterBar::query() const { return search_box_->text().trimmed(); }

void FilterBar::SetQuery(const QString& query) {
  if (search_box_->text() == query) {
    return;
  }
  search_box_->setText(query);
}

void FilterBar::SetDebounceIntervalMs(int interval_ms) {
  debounce_interval_ms_ = std::max(0, interval_ms);
  if (debounce_timer_->isActive()) {
    ScheduleQueryChanged();
  }
}

void FilterBar::AddFilterWidget(QWidget* widget, int stretch) {
  if (widget == nullptr || filters_layout_->indexOf(widget) >= 0) {
    return;
  }
  widget->setParent(this);
  ApplyWidgetTheme(widget, tone_, true);
  filters_layout_->addWidget(widget, std::max(0, stretch));
}

QWidget* FilterBar::TakeFilterWidget(QWidget* widget) {
  if (widget == nullptr || filters_layout_->indexOf(widget) < 0) {
    return nullptr;
  }
  filters_layout_->removeWidget(widget);
  widget->hide();
  widget->setParent(nullptr);
  return widget;
}

int FilterBar::filter_count() const noexcept {
  return filters_layout_->count();
}

void FilterBar::SetTone(style::Tone tone) {
  tone_ = tone;
  search_box_->SetTone(tone);
  clear_button_->SetTone(tone);
  for (int index = 0; index < filters_layout_->count(); ++index) {
    if (QWidget* widget = filters_layout_->itemAt(index)->widget();
        widget != nullptr) {
      ApplyWidgetTheme(widget, tone, true);
    }
  }
  update();
}

void FilterBar::Clear() {
  debounce_timer_->stop();
  {
    const QSignalBlocker blocker(search_box_);
    search_box_->clear();
  }
  UpdateClearButton();
  QPointer<FilterBar> guard(this);
  emit QueryChanged(QString());
  if (guard != nullptr) {
    emit Cleared();
  }
}

void FilterBar::Submit() {
  debounce_timer_->stop();
  const QString submitted_query = query();
  QPointer<FilterBar> guard(this);
  emit QueryChanged(submitted_query);
  if (guard != nullptr) {
    emit SearchRequested(submitted_query);
  }
}

void FilterBar::hideEvent(QHideEvent* event) {
  const bool had_pending_change = debounce_timer_->isActive();
  const QString pending_query = query();
  debounce_timer_->stop();
  QPointer<FilterBar> guard(this);
  QFrame::hideEvent(event);
  if (guard != nullptr && had_pending_change) {
    emit QueryChanged(pending_query);
  }
}

void FilterBar::ScheduleQueryChanged() {
  if (!isVisible() || debounce_interval_ms_ <= 0) {
    debounce_timer_->stop();
    emit QueryChanged(query());
    return;
  }
  debounce_timer_->start(debounce_interval_ms_);
}

void FilterBar::UpdateClearButton() {
  clear_button_->setEnabled(!search_box_->text().isEmpty());
}

}  // namespace autoviz::widget
