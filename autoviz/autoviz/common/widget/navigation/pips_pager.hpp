/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QRectF>
#include <QVector>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QEvent;
class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QResizeEvent;

namespace autoviz::widget {

// Compact page selector for carousels and small result sets. For large data
// tables use Pagination instead.
class PipsPager final : public QWidget {
  Q_OBJECT

 public:
  explicit PipsPager(QWidget* parent = nullptr);

  int page_count() const noexcept { return page_count_; }
  void SetPageCount(int count);
  int current_page() const noexcept { return current_page_; }
  void SetCurrentPage(int page);
  int maximum_visible_pips() const noexcept { return maximum_visible_pips_; }
  void SetMaximumVisiblePips(int count);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void CurrentPageChanged(int page);
  void PageActivated(int page);

 protected:
  void changeEvent(QEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  void ActivatePage(int page);
  void RebuildGeometry();
  void UpdateAccessibility();
  int HitTest(const QPointF& position) const;

  QVector<int> visible_pages_;
  QVector<QRectF> pip_rects_;
  int page_count_ = 0;
  int current_page_ = -1;
  int maximum_visible_pips_ = 7;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
