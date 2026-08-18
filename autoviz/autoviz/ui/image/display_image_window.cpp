/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/display_image_window.hpp"

#include <QVBoxLayout>

#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/image/image_view_widget.hpp"

namespace autoviz {
namespace image {
namespace {

constexpr int kDefaultWidth = 480;
constexpr int kDefaultHeight = 360;

}  // namespace

DisplayImageWindow::DisplayImageWindow(const QString& display_name,
                                       QWidget* parent)
    : QWidget(parent, Qt::Window) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  view_ = new ImageViewWidget(this);
  view_->setBackgroundColor(Qt::white);
  view_->setStatusText(tr("No Image"));
  layout->addWidget(view_);
  setWindowIcon(IconLoader::panelIcon(QStringLiteral("PanelImage")));
  resize(kDefaultWidth, kDefaultHeight);
  setDisplayName(display_name);
}

void DisplayImageWindow::setDisplayName(const QString& display_name) {
  setWindowTitle(display_name.trimmed().isEmpty() ? tr("Image")
                                                  : display_name.trimmed());
}

void DisplayImageWindow::setFrame(const QImage& image) {
  if (image.isNull() || view_ == nullptr) {
    return;
  }
  view_->setFrame(image);
}

}  // namespace image
}  // namespace autoviz
