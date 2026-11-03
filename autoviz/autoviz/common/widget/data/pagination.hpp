/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QComboBox;
class QLabel;

namespace autoviz::widget {

class Button;

// Model-friendly pagination control. Pages are zero-based in the API and
// displayed one-based to users. It never owns or slices the backing model.
class Pagination final : public QWidget {
  Q_OBJECT
  Q_PROPERTY(qint64 totalItems READ total_items WRITE SetTotalItems NOTIFY
                 TotalItemsChanged)
  Q_PROPERTY(
      int pageSize READ page_size WRITE SetPageSize NOTIFY PageSizeChanged)
  Q_PROPERTY(qint64 currentPage READ current_page WRITE SetCurrentPage NOTIFY
                 CurrentPageChanged)

 public:
  explicit Pagination(QWidget* parent = nullptr);

  qint64 total_items() const noexcept { return total_items_; }
  int page_size() const noexcept { return page_size_; }
  qint64 current_page() const noexcept { return current_page_; }
  qint64 page_count() const noexcept;
  qint64 offset() const noexcept;

  const QVector<int>& page_sizes() const noexcept { return page_sizes_; }
  void SetPageSizes(QVector<int> page_sizes);

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 public slots:
  void SetTotalItems(qint64 total_items);
  void SetPageSize(int page_size);
  void SetCurrentPage(qint64 page);
  void GoToFirstPage();
  void GoToLastPage();
  void GoToPreviousPage();
  void GoToNextPage();

 signals:
  void TotalItemsChanged(qint64 total_items);
  void PageSizeChanged(int page_size);
  void CurrentPageChanged(qint64 page);
  void PageRequested(qint64 page, int page_size);

 protected:
  void keyPressEvent(QKeyEvent* event) override;

 private:
  qint64 ClampPage(qint64 page) const noexcept;
  void RebuildPageSizes();
  void UpdateControls();

  Button* previous_button_ = nullptr;
  Button* next_button_ = nullptr;
  QLabel* range_label_ = nullptr;
  QLabel* page_label_ = nullptr;
  QComboBox* page_size_box_ = nullptr;
  QVector<int> page_sizes_ = {25, 50, 100, 200};
  style::Tone tone_ = style::Tone::Shell;
  qint64 total_items_ = 0;
  int page_size_ = 50;
  qint64 current_page_ = 0;
};

}  // namespace autoviz::widget
