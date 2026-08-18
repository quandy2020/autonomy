/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>
#include <QString>
#include <QWidget>

namespace autoviz {
namespace image {

class ImageViewWidget;

/**
 * Window that belongs to one Image display, like the associated widget RViz2
 * opens next to its Image display: it shows only that display's frames and
 * follows the display's name and enabled state.
 *
 * It is a plain top-level widget rather than a PanelDockWidget on purpose, so
 * it stays out of the Panels menu and the saved dock layout.
 */
class DisplayImageWindow : public QWidget {
  Q_OBJECT

 public:
  explicit DisplayImageWindow(const QString& display_name,
                              QWidget* parent = nullptr);

  void setDisplayName(const QString& display_name);
  void setFrame(const QImage& image);

 private:
  ImageViewWidget* view_ = nullptr;
};

}  // namespace image
}  // namespace autoviz
