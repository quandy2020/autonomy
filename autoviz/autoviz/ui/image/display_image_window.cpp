/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/display_image_window.hpp"

#include <QCloseEvent>
#include <QVBoxLayout>

#include "autoviz/ui/app/icon_loader.hpp"
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
  setAttribute(Qt::WA_DeleteOnClose, false);
  resize(kDefaultWidth, kDefaultHeight);
  setDisplayName(display_name);
}

void DisplayImageWindow::setDisplayName(const QString& display_name) {
  display_name_ = display_name.trimmed();
  setWindowTitle(display_name_.isEmpty() ? tr("Image") : display_name_);
}

void DisplayImageWindow::setFrame(const QImage& image) {
  if (image.isNull() || view_ == nullptr) {
    return;
  }
  view_->setFrame(image);
}

void DisplayImageWindow::closeQuietly() {
  suppress_close_signal_ = true;
  close();
  suppress_close_signal_ = false;
}

void DisplayImageWindow::closeEvent(QCloseEvent* event) {
  if (!suppress_close_signal_) {
    emit closedByUser();
  }
  QWidget::closeEvent(event);
}

}  // namespace image
}  // namespace autoviz
