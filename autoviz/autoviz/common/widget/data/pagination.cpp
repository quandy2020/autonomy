/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/pagination.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPointer>
#include <QSignalBlocker>
#include <QVariant>
#include <algorithm>
#include <utility>

#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/button.hpp"

namespace autoviz::widget {

Pagination::Pagination(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizPagination"));
  setFocusPolicy(Qt::StrongFocus);
  setAccessibleName(tr("Pagination"));
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  auto* layout = new QHBoxLayout(this);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(tokens.spacing_sm);

  range_label_ = new QLabel(this);
  range_label_->setAccessibleName(tr("Visible item range"));
  layout->addWidget(range_label_);
  layout->addStretch(1);

  auto* page_size_label = new QLabel(tr("Rows"), this);
  layout->addWidget(page_size_label);

  page_size_box_ = new QComboBox(this);
  page_size_box_->setAccessibleName(tr("Rows per page"));
  page_size_box_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  layout->addWidget(page_size_box_);
  page_size_label->setBuddy(page_size_box_);

  previous_button_ = new Button(QStringLiteral("‹"), this);
  previous_button_->SetButtonSize(Button::Size::kSmall);
  previous_button_->SetVariant(Button::Variant::kSubtle);
  previous_button_->setAccessibleName(tr("Previous page"));
  previous_button_->setToolTip(tr("Previous page"));
  layout->addWidget(previous_button_);

  page_label_ = new QLabel(this);
  page_label_->setAlignment(Qt::AlignCenter);
  page_label_->setMinimumWidth(72);
  page_label_->setAccessibleName(tr("Current page"));
  layout->addWidget(page_label_);

  next_button_ = new Button(QStringLiteral("›"), this);
  next_button_->SetButtonSize(Button::Size::kSmall);
  next_button_->SetVariant(Button::Variant::kSubtle);
  next_button_->setAccessibleName(tr("Next page"));
  next_button_->setToolTip(tr("Next page"));
  layout->addWidget(next_button_);

  connect(previous_button_, &QAbstractButton::clicked, this,
          &Pagination::GoToPreviousPage);
  connect(next_button_, &QAbstractButton::clicked, this,
          &Pagination::GoToNextPage);
  connect(page_size_box_, &QComboBox::currentIndexChanged, this,
          [this](int index) {
            SetPageSize(page_size_box_->itemData(index).toInt());
          });

  RebuildPageSizes();
  SetTone(tone_);
  UpdateControls();
}

qint64 Pagination::page_count() const noexcept {
  if (total_items_ <= 0 || page_size_ <= 0) {
    return 0;
  }
  return 1 + (total_items_ - 1) / page_size_;
}

qint64 Pagination::offset() const noexcept {
  return current_page_ * page_size_;
}

void Pagination::SetPageSizes(QVector<int> page_sizes) {
  page_sizes.erase(std::remove_if(page_sizes.begin(), page_sizes.end(),
                                  [](int size) { return size <= 0; }),
                   page_sizes.end());
  std::sort(page_sizes.begin(), page_sizes.end());
  page_sizes.erase(std::unique(page_sizes.begin(), page_sizes.end()),
                   page_sizes.end());
  if (page_sizes.isEmpty()) {
    page_sizes.push_back(page_size_);
  }
  if (!page_sizes.contains(page_size_)) {
    page_sizes.push_back(page_size_);
    std::sort(page_sizes.begin(), page_sizes.end());
  }
  if (page_sizes_ == page_sizes) {
    return;
  }
  page_sizes_ = std::move(page_sizes);
  RebuildPageSizes();
}

void Pagination::SetTone(style::Tone tone) {
  tone_ = tone;
  previous_button_->SetTone(tone);
  next_button_->SetTone(tone);
  ApplyWidgetTheme(page_size_box_, tone, false);
  update();
}

void Pagination::SetTotalItems(qint64 total_items) {
  total_items = std::max<qint64>(0, total_items);
  if (total_items_ == total_items) {
    return;
  }
  total_items_ = total_items;
  const qint64 bounded_page = ClampPage(current_page_);
  const bool page_changed = current_page_ != bounded_page;
  current_page_ = bounded_page;
  UpdateControls();
  const qint64 emitted_total = total_items_;
  const qint64 requested_page = current_page_;
  const int requested_size = page_size_;
  QPointer<Pagination> guard(this);
  emit TotalItemsChanged(emitted_total);
  if (guard != nullptr && page_changed) {
    emit CurrentPageChanged(requested_page);
  }
  if (guard != nullptr) {
    emit PageRequested(requested_page, requested_size);
  }
}

void Pagination::SetPageSize(int page_size) {
  if (page_size <= 0 || page_size_ == page_size) {
    return;
  }
  const qint64 first_visible_item = offset();
  page_size_ = page_size;
  if (!page_sizes_.contains(page_size_)) {
    page_sizes_.push_back(page_size_);
    std::sort(page_sizes_.begin(), page_sizes_.end());
    RebuildPageSizes();
  }
  const qint64 new_page = ClampPage(first_visible_item / page_size_);
  const bool page_changed = current_page_ != new_page;
  current_page_ = new_page;
  UpdateControls();
  const int emitted_page_size = page_size_;
  const qint64 requested_page = current_page_;
  QPointer<Pagination> guard(this);
  emit PageSizeChanged(emitted_page_size);
  if (guard != nullptr && page_changed) {
    emit CurrentPageChanged(requested_page);
  }
  if (guard != nullptr) {
    emit PageRequested(requested_page, emitted_page_size);
  }
}

void Pagination::SetCurrentPage(qint64 page) {
  const qint64 bounded_page = ClampPage(page);
  if (current_page_ == bounded_page) {
    return;
  }
  current_page_ = bounded_page;
  UpdateControls();
  const qint64 requested_page = current_page_;
  const int requested_size = page_size_;
  QPointer<Pagination> guard(this);
  emit CurrentPageChanged(current_page_);
  if (guard != nullptr) {
    emit PageRequested(requested_page, requested_size);
  }
}

void Pagination::GoToFirstPage() { SetCurrentPage(0); }

void Pagination::GoToLastPage() {
  SetCurrentPage(std::max<qint64>(0, page_count() - 1));
}

void Pagination::GoToPreviousPage() { SetCurrentPage(current_page_ - 1); }

void Pagination::GoToNextPage() { SetCurrentPage(current_page_ + 1); }

void Pagination::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Left:
    case Qt::Key_PageUp:
      GoToPreviousPage();
      event->accept();
      return;
    case Qt::Key_Right:
    case Qt::Key_PageDown:
      GoToNextPage();
      event->accept();
      return;
    case Qt::Key_Home:
      GoToFirstPage();
      event->accept();
      return;
    case Qt::Key_End:
      GoToLastPage();
      event->accept();
      return;
    default:
      break;
  }
  QWidget::keyPressEvent(event);
}

qint64 Pagination::ClampPage(qint64 page) const noexcept {
  const qint64 count = page_count();
  return count <= 0 ? 0 : qBound<qint64>(0, page, count - 1);
}

void Pagination::RebuildPageSizes() {
  const QSignalBlocker blocker(page_size_box_);
  page_size_box_->clear();
  for (int size : page_sizes_) {
    page_size_box_->addItem(QString::number(size), size);
  }
  page_size_box_->setCurrentIndex(page_size_box_->findData(page_size_));
}

void Pagination::UpdateControls() {
  const qint64 count = page_count();
  const bool has_items = count > 0;
  previous_button_->setEnabled(has_items && current_page_ > 0);
  next_button_->setEnabled(has_items && current_page_ + 1 < count);
  page_label_->setText(has_items
                           ? tr("%1 / %2").arg(current_page_ + 1).arg(count)
                           : tr("0 / 0"));

  if (!has_items) {
    range_label_->setText(tr("0 items"));
    range_label_->setAccessibleDescription(tr("No items"));
    return;
  }
  const qint64 page_offset = offset();
  const qint64 first = page_offset + 1;
  const qint64 visible_count =
      std::min<qint64>(page_size_, total_items_ - page_offset);
  const qint64 last = page_offset + visible_count;
  const QString range =
      tr("%1–%2 of %3").arg(first).arg(last).arg(total_items_);
  range_label_->setText(range);
  range_label_->setAccessibleDescription(range);
}

}  // namespace autoviz::widget
