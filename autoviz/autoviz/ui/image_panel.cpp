/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image_panel.hpp"

#include <QPixmap>
#include <QVBoxLayout>

namespace autoviz {

ImagePanel::ImagePanel(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  source_label_ = new QLabel(tr("No image"), this);
  source_label_->setWordWrap(true);
  layout->addWidget(source_label_);

  image_label_ = new QLabel(this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumHeight(180);
  image_label_->setScaledContents(true);
  layout->addWidget(image_label_, 1);
}

void ImagePanel::setImage(const QString& source, const QImage& image) {
  if (image.isNull()) {
    source_label_->setText(tr("No image"));
    image_label_->clear();
    return;
  }
  source_label_->setText(source);
  image_label_->setPixmap(QPixmap::fromImage(image));
}

}  // namespace autoviz
