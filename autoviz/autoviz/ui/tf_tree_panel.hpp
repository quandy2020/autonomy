/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/transform/buffer.hpp"

class QTreeWidget;

namespace autoviz {

class TfTreePanel : public QWidget {
  Q_OBJECT

 public:
  explicit TfTreePanel(autoviz::transform::Buffer* tf_buffer,
                      QWidget* parent = nullptr);

 public slots:
  void refresh();

 private:
  void setupUi();

  autoviz::transform::Buffer* tf_buffer_ = nullptr;
  QTreeWidget* tree_ = nullptr;
};

}  // namespace autoviz
