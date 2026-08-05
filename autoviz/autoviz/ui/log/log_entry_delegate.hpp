/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QStyledItemDelegate>

namespace autoviz {
namespace log_panel {

class LogEntryDelegate : public QStyledItemDelegate {
 public:
  explicit LogEntryDelegate(QObject* parent = nullptr);

  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override;
  QSize sizeHint(const QStyleOptionViewItem& option,
                 const QModelIndex& index) const override;
};

}  // namespace log_panel
}  // namespace autoviz
