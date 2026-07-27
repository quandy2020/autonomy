/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QTextBrowser>
#include <QWidget>

namespace autoviz {

class HelpPanel : public QWidget {
  Q_OBJECT

 public:
  explicit HelpPanel(QWidget* parent = nullptr);
};

}  // namespace autoviz
