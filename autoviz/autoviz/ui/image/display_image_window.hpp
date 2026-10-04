/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_image_window.hpp
 * @brief Top-level window that mirrors a single Image display's frames.
 *
 * Opened beside an Image display (RViz2-style associated widget): shows only
 * that display's decoded frames and tracks its name / enabled state. Kept as a
 * plain top-level @c QWidget so it stays out of the Panels menu and saved dock
 * layout.
 *
 * @see ImageViewWidget
 * @see ImagePanel
 */

#pragma once

#include <QImage>
#include <QString>
#include <QWidget>

class QCloseEvent;

namespace autoviz {
namespace image {

class ImageViewWidget;

/**
 * @class DisplayImageWindow
 * @brief Detached viewer window owned by one Image display instance.
 *
 * Hosts an @ref ImageViewWidget that receives frames via @ref setFrame().
 * Title tracks the display name through @ref setDisplayName().
 *
 * Shown when an Image display is enabled (RViz2 @c setAssociatedWidget
 * analogue). Closing the window emits @ref closedByUser() so the frame can
 * disable the owning display.
 *
 * @note Not a @c PanelDockWidget — intentionally excluded from panel chrome
 *       and session dock persistence.
 */
class DisplayImageWindow : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the window and embeds an @ref ImageViewWidget.
   *
   * @param display_name Initial window title / display label.
   * @param parent Qt parent (typically the main frame; still a top-level
   *        window via @c Qt::Window).
   */
  explicit DisplayImageWindow(const QString& display_name,
                              QWidget* parent = nullptr);

  /**
   * @brief Updates the window title to match the owning display's name.
   *
   * @param display_name New display name shown in the title bar.
   */
  void setDisplayName(const QString& display_name);

  /**
   * @brief Returns the display name this window is bound to.
   */
  QString displayName() const { return display_name_; }

  /**
   * @brief Pushes a decoded frame into the embedded view.
   *
   * @param image RGB frame to display (copied into the view widget).
   * @see ImageViewWidget::setFrame()
   */
  void setFrame(const QImage& image);

  /**
   * @brief Closes without emitting @ref closedByUser() (programmatic teardown).
   */
  void closeQuietly();

 Q_SIGNALS:
  /**
   * @brief Emitted when the user closes the window (title-bar / Alt+F4).
   *
   * Listeners should disable the matching Image display.
   */
  void closedByUser();

 protected:
  /**
   * @brief Emits @ref closedByUser() unless @ref closeQuietly() is in progress.
   */
  void closeEvent(QCloseEvent* event) override;

 private:
  /** Bound Displays-tree name (window title source). */
  QString display_name_;

  /** Embedded pan/zoom image view (owned via Qt parentship). */
  ImageViewWidget* view_ = nullptr;

  /** When true, @ref closeEvent skips @ref closedByUser(). */
  bool suppress_close_signal_ = false;
};

}  // namespace image
}  // namespace autoviz
