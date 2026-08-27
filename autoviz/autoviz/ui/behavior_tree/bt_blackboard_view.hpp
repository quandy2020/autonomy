/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QString>
#include <QWidget>

class QTableWidget;

namespace autoviz {
namespace behavior_tree {

/** Simple key/value table for behavior-tree blackboard entries. */
class BtBlackboardView : public QWidget {
  Q_OBJECT

 public:
  explicit BtBlackboardView(QWidget* parent = nullptr);

  void setEntries(const QHash<QString, QString>& entries);
  void clear();

 private:
  QTableWidget* table_ = nullptr;
};

}  // namespace behavior_tree
}  // namespace autoviz
