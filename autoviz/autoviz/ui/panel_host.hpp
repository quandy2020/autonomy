/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel_host.hpp
 * @brief Nested QMainWindow that tiles main panels in a grippable splitter grid.
 *
 * @ref MainPanelHost is the center column of @ref VisualizationFrame. Visible
 * main panels are laid out in a @c QSplitter tree (not @c QMainWindow dock
 * splits) so separators stay usable on nested hosts.
 *
 * @see FrameLayout
 * @see PanelDockWidget
 * @see FrameLayout::tileCenterPanels()
 */

#pragma once

#include <functional>

#include <QByteArray>
#include <QList>
#include <QMainWindow>
#include <QString>

class QDockWidget;
class QResizeEvent;
class QSplitter;

namespace autoviz {

/**
 * @class MainPanelHost
 * @brief Central column host for Foxglove-style main panel tiling.
 *
 * ## Layout model
 *
 * Nested @c QSplitter tree mirroring Foxglove's react-mosaic binary layout:
 * Split right / down replace a leaf in place (`first` = original,
 * `second` = duplicate) without rebuilding the whole grid.
 *
 * @code
 * MainPanelHost (QMainWindow, no menu/toolbar)
 *   └── root_splitter_
 *         ├── dock A | dock A'
 *         └── nested (column)
 *               ├── dock B
 *               └── dock B'
 * @endcode
 *
 * Hidden docks are detached from the splitter and kept invisible without
 * deleting them, so session restore can re-show them quickly.
 *
 * @note @ref minimumSizeHint() / @ref sizeHint() return small values so the
 *       nested host does not inflate @ref VisualizationFrame minimum size.
 *
 * @see FrameLayout::setupMainPanelHost()
 * @see PreferredGridSize()
 */
class MainPanelHost : public QMainWindow {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty host with a root splitter.
   * @param parent Typically the central container of @ref VisualizationFrame.
   */
  explicit MainPanelHost(QWidget* parent = nullptr);

  /**
   * @brief Preferred rows×cols for a panel count (4/6/9/16-style grids).
   *
   * @param count Number of visible panels to arrange.
   * @param[out] rows Preferred row count (must be non-null).
   * @param[out] cols Preferred column count (must be non-null).
   */
  static void PreferredGridSize(int count, int* rows, int* cols);

  /**
   * @brief Place @p visible panels into a resizable splitter grid; @p hidden
   *        are detached and kept invisible.
   *
   * Rebuilds the splitter tree. Safe to call repeatedly; uses @c tiling_ as a
   * re-entrancy guard.
   *
   * @param visible Docks to show in the grid (order preserved where possible).
   * @param hidden Docks to detach and hide without deleting.
   */
  void tilePanels(const QList<QDockWidget*>& visible,
                  const QList<QDockWidget*>& hidden);

  /**
   * @brief @c true if @p dock is currently managed by this host's splitter.
   * @param dock Dock to query.
   */
  bool hostsPanel(const QDockWidget* dock) const;

  /**
   * @brief Attach a single panel (e.g. after show) without a full retile.
   * @param dock Dock to add to the current splitter layout.
   */
  void addPanel(QDockWidget* dock);

  /**
   * @brief Detach a panel from the splitter (does not delete).
   * @param dock Dock to remove from management.
   */
  void removePanel(QDockWidget* dock);

  /**
   * @brief Foxglove-style in-place mosaic split of @p first.
   *
   * Replaces the leaf hosting @p first with a binary split node
   * `{ first, second, direction }` (react-mosaic / Foxglove Studio):
   * - @c Qt::Horizontal ("row") — Split right: original left, @p second right
   * - @c Qt::Vertical ("column") — Split down: original top, @p second bottom
   *
   * Sibling panes keep their geometry; only the split leaf is halved (~50/50).
   *
   * @param first Existing hosted dock (stays as mosaic @c first).
   * @param second New dock placed as mosaic @c second (often a duplicate).
   * @param orientation Split direction.
   */
  void splitPanel(QDockWidget* first, QDockWidget* second,
                  Qt::Orientation orientation);

  /**
   * @brief Equalize current splitter sizes (window resize helper).
   */
  void syncHorizontalDockLayout();

  /**
   * @brief Serializes the center mosaic (splitter tree + leaf dock names).
   *
   * Format: @c mosaic:v1: + compact JSON. Replaces legacy
   * @c QMainWindow::saveState() blobs which do not capture splitter layout.
   *
   * @return Encoded layout bytes (may be empty when there are no leaves).
   */
  QByteArray saveMosaicLayout() const;

  /**
   * @brief Rebuilds the splitter tree from @ref saveMosaicLayout() output.
   *
   * @param encoded Bytes from @ref saveMosaicLayout() (or legacy ignored).
   * @param resolve Maps dock @c objectName → existing @c QDockWidget
   *        (must not create widgets; return null to skip a leaf).
   * @return @c true when a mosaic:v1 payload was applied successfully.
   */
  bool restoreMosaicLayout(
      const QByteArray& encoded,
      const std::function<QDockWidget*(const QString& object_name)>& resolve);

  /**
   * @brief Nested host must not push large minimums into VisualizationFrame.
   * @return Compact minimum size.
   */
  QSize minimumSizeHint() const override;

  /**
   * @brief Preferred size hint kept intentionally small.
   * @return Compact size hint.
   */
  QSize sizeHint() const override;

 protected:
  /**
   * @brief Relays resize and may equalize splitter sizes.
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

 private:
  /**
   * @brief Destroys nested splitters and clears hosted widget parenting.
   */
  void clearSplitterTree();

  /**
   * @brief Equalizes sizes of the top-level splitter children.
   */
  void equalizeTopLevel();

  /**
   * @brief Collects docks currently attached under @c root_splitter_.
   * @return List of hosted docks.
   */
  QList<QDockWidget*> hostedPanels() const;

  /**
   * @brief Parks a dock under @ref parking_ (never as a QMainWindow child).
   *
   * @c QDockWidget::setParent(QMainWindow*) engages Qt dock machinery and can
   * corrupt the widget heap when the dock uses @c NoDockWidgetArea.
   */
  void parkDock(QDockWidget* dock);

  /** Root of the nested splitter tree (central widget). */
  QSplitter* root_splitter_ = nullptr;

  /**
   * Hidden plain @c QWidget used to hold docks detached from the mosaic.
   * Must not be a @c QMainWindow — see @ref parkDock().
   */
  QWidget* parking_ = nullptr;

  /** Re-entrancy guard while @ref tilePanels() rebuilds the tree. */
  bool tiling_ = false;
};

}  // namespace autoviz
