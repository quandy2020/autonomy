/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

namespace autoviz {

/** Foxglove-style global Variables sidebar (placeholder). */
class VariablesPanel : public QWidget {
  Q_OBJECT

 public:
  explicit VariablesPanel(QWidget* parent = nullptr);
};

}  // namespace autoviz
