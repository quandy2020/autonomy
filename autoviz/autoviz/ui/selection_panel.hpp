/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QListWidget>
#include <QWidget>

#include "autoviz/common/selection.hpp"

namespace autoviz {

class SelectionPanel : public QWidget {
  Q_OBJECT

 public:
  explicit SelectionPanel(QWidget* parent = nullptr);

  void setSelections(const std::vector<common::SelectionEntry>& entries);

 private:
  QListWidget* list_ = nullptr;
};

}  // namespace autoviz
