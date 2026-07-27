/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>
#include <QLabel>
#include <QWidget>

namespace autoviz {

class ImagePanel : public QWidget {
  Q_OBJECT

 public:
  explicit ImagePanel(QWidget* parent = nullptr);

  void setImage(const QString& source, const QImage& image);

 private:
  QLabel* source_label_ = nullptr;
  QLabel* image_label_ = nullptr;
};

}  // namespace autoviz
