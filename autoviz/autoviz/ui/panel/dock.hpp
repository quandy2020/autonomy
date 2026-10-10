/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file dock.hpp
 * @brief RViz-style dock panel with custom title bar and collapsed state.
 *
 * @ref PanelDockWidget replaces the stock @c QDockWidget title bar with icon,
 * title, optional Foxglove title-bar tools, and close. Supports forced hide
 * (layout hide-left/right) without losing the user’s requested visibility.
 *
 * @see FrameLayout
 * @see CreatePanelTitleBarTools()
 * @see IconLoader::applyDockPanelChrome()
 */

#pragma once

#include <QDockWidget>
#include <QLabel>
#include <QPoint>

class QMouseEvent;
class QToolButton;
class QMainWindow;

namespace autoviz {

/**
 * @class PanelDockWidget
 * @brief RViz-style dock panel with custom title bar and collapsed state.
 *
 * ## Title bar
 *
 * Icon + title label + optional @ref setTitleBarTools() host + close. Title-bar
 * press emits @ref activated(); drag emits @ref titleDragStarted() /
 * @ref titleDragFinished() for center-tile suppression.
 *
 * ## Visibility
 *
 * @ref overrideVisibility() forces hidden (sidebar hide) while remembering
 * @c requested_visibility_ so restore can reopen only docks the user wanted.
 *
 * ## Fixed height
 *
 * @ref setFixedContentHeight() locks docks like the bottom Time bar to
 * title + content height.
 *
 * @see FrameLayout::hideLeftDock()
 * @see MainPanelHost
 */
class PanelDockWidget : public QDockWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs a dock with a custom title bar labelled @p name.
   *
   * @param name Initial window / panel title.
   * @param parent Parent @c QMainWindow or host.
   */
  explicit PanelDockWidget(const QString& name, QWidget* parent = nullptr);

  /**
   * @brief Sets the dock content widget (replaces any previous child).
   * @param child Content widget; takes ownership via Qt parenting.
   */
  void setContentWidget(QWidget* child);

  /**
   * @brief Collapses or expands the content area (title bar remains).
   * @param collapsed @c true to hide content.
   * @see isCollapsed()
   */
  void setCollapsed(bool collapsed);

  /**
   * @brief @c true when content is collapsed.
   */
  bool isCollapsed() const { return collapsed_; }

  /**
   * @brief Sets the title-bar icon.
   * @param icon Panel icon (16×16 typical).
   */
  void setPanelIcon(const QIcon& icon);

  /**
   * @brief Sets the title-bar text (and window title).
   * @param title Display title.
   */
  void setPanelTitle(const QString& title);

  /**
   * @brief Foxglove-style tools shown in the title bar before the close button.
   * @param tools Tools host widget (may be @c nullptr to clear).
   * @see CreatePanelTitleBarTools()
   */
  void setTitleBarTools(QWidget* tools);

  /**
   * @brief Lock dock height to title bar + fixed content (e.g. bottom Time bar).
   * @param height Content height in pixels (0 clears the lock).
   * @see enforceFixedHeight()
   */
  void setFixedContentHeight(int height);

  /**
   * @brief Re-applies the fixed height constraint after layout changes.
   */
  void enforceFixedHeight();

  /**
   * @brief Force-hides or restores visibility without forgetting user intent.
   *
   * @param hidden @c true to force hide; @c false to restore
   *        @c requested_visibility_.
   */
  void overrideVisibility(bool hidden);

  /**
   * @brief User-requested visibility (ignores ancestor @c isVisible() chain).
   *
   * Before the main window is shown, @c QWidget::isVisible() is false even
   * after @c show(). Mosaic tiling must use this flag or the center stays empty.
   */
  bool requestedVisible() const { return requested_visibility_; }

  /**
   * @brief @c true while the user is dragging this dock via the title bar.
   */
  bool isTitleDragActive() const { return title_drag_active_; }

 protected:
  /**
   * @brief Emits @ref closed() then accepts the close.
   * @param event Close event.
   */
  void closeEvent(QCloseEvent* event) override;

  /**
   * @brief Tracks requested visibility and honors @c forced_hidden_.
   * @param visible Requested visibility.
   */
  void setVisible(bool visible) override;

  /**
   * @brief Re-applies fixed height hooks when shown.
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

  /**
   * @brief Enforces fixed height during resize when configured.
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

  /**
   * @brief Title-bar mouse filter for activate / drag tracking.
   *
   * @param watched Filtered object.
   * @param event Event under consideration.
   * @return @c true if consumed.
   */
  bool eventFilter(QObject* watched, QEvent* event) override;

  /**
   * @brief Handles style / layout change events on the dock.
   * @param event Change event.
   */
  void changeEvent(QEvent* event) override;

 private slots:
  /**
   * @brief Marks the dock disposed and @c deleteLater() when content is destroyed.
   * @param child Destroyed content widget (from @ref setContentWidget).
   */
  void onChildDestroyed(QObject* child);

 signals:
  /**
   * @brief Emitted when the dock is closed by the user.
   */
  void closed();

  /**
   * @brief Emitted when the user clicks the panel title bar.
   *
   * Used by @ref FrameLayout / @ref FramePanels to track the last-active dock.
   */
  void activated();

  /**
   * @brief Emitted when a title-bar drag begins.
   */
  void titleDragStarted();

  /**
   * @brief Emitted when a title-bar drag ends.
   */
  void titleDragFinished();

 private:
  /**
   * @brief Applies min/max/fixed height from @c fixed_content_height_.
   */
  void applyFixedContentHeight();

  /**
   * @brief Installs event filters / hooks needed for fixed-height docks.
   */
  void installFixedHeightHooks();

  /**
   * @brief Computes total dock height (title + fixed content).
   * @return Height in pixels.
   */
  int fixedDockHeight() const;

  /**
   * @brief Finds the owning @c QMainWindow ancestor.
   * @return Main window, or @c nullptr.
   */
  QMainWindow* mainWindow() const;

  /**
   * @brief Clears drag state and emits @ref titleDragFinished() if needed.
   */
  void endTitleDrag();

  /** Content currently collapsed. */
  bool collapsed_ = false;

  /** Last user-requested visibility (before force-hide). */
  bool requested_visibility_ = true;

  /** Layout force-hide (hide left/right docks). */
  bool forced_hidden_ = false;

  /** Title-bar drag in progress. */
  bool title_drag_active_ = false;

  /** Title-bar press armed (may become a drag). */
  bool title_press_active_ = false;

  /** Global position at title-bar press. */
  QPoint title_press_pos_;

  /** Fixed content height in px (0 = unlocked). */
  int fixed_content_height_ = 0;

  /** Whether fixed-height hooks are installed. */
  bool fixed_height_hooks_installed_ = false;

  /** Re-entrancy guard while enforcing height. */
  bool height_enforcing_ = false;

  /** Custom title bar widget. */
  QWidget* title_bar_ = nullptr;

  /** Title-bar icon label. */
  QLabel* icon_label_ = nullptr;

  /** Title-bar text label. */
  QLabel* title_label_ = nullptr;

  /** Host for optional title-bar tools. */
  QWidget* title_tools_host_ = nullptr;
};

}  // namespace autoviz
