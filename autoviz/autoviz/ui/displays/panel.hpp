/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel.hpp
 * @brief Displays panel — RViz2-style display list, global options, and help.
 *
 * The Displays panel is hosted in the left sidebar dock. It binds to
 * @ref common::VisualizationManager for the live display list, Fixed Frame,
 * background color, and status aggregation, and embeds a
 * @ref DisplayTreeWidget for drag-and-drop regrouping.
 *
 * @see DisplayTreeWidget
 * @see DisplayTreeDelegate
 * @see AddDisplayDialog
 * @see common::VisualizationManager
 */

#pragma once

#include <memory>

#include <QSplitter>
#include <QTreeWidget>
#include <QWidget>

#include "autoviz/ui/displays/tree_delegate.hpp"
#include "autoviz/ui/displays/tree.hpp"

class QPaintEvent;
class QPushButton;
class QTextBrowser;
class QTimer;
class QTreeWidgetItem;

namespace autoviz {
namespace common {
class VisualizationManager;
}
namespace display {
class Display;
class TfDisplay;
}

/**
 * @class DisplaysPanel
 * @brief Sidebar panel for managing Displays, Global Options, and status help.
 *
 * ## Layout
 *
 * @code
 * ┌─────────────────────────────────────┐
 * │ ▾ Global Options                    │
 * │     Fixed Frame              map    │
 * │     Background Color         …      │
 * │ ▾ Global Status                     │
 * │ ▾ TF                         /tf    │
 * │     … properties …                  │
 * ├─────────────────────────────────────┤
 * │ (help browser for selected row)     │
 * ├─────────────────────────────────────┤
 * │ [Add] [Duplicate] [Rename] [Remove] │
 * └─────────────────────────────────────┘
 * @endcode
 *
 * ## Data flow
 *
 * - **Out:** property edits, Add/Duplicate/Rename/Remove, and Fixed Frame /
 *   background changes call into VisualizationManager and emit
 *   @ref displaysChanged() / @ref fixedFrameChanged() /
 *   @ref backgroundColorChanged().
 * - **In:** @ref refresh() rebuilds the tree; @ref refreshStatus() updates
 *   status icons and the channel delegate without a full rebuild;
 *   @ref setLiveUpdatesPaused() gates the TF pose timer.
 *
 * ## Styling
 *
 * Draws a frosted card via theme helpers in @ref paintEvent(); tree value
 * column uses @ref DisplayTreeDelegate.
 *
 * @note The panel shares ownership of VisualizationManager via
 *       @c std::shared_ptr; it does not own individual Display instances.
 *
 * @see DisplayTreeWidget
 * @see AddDisplayDialog
 * @see FramePanels
 */
class DisplaysPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the Displays panel and builds the initial property tree.
   *
   * @param manager Shared visualization manager owning the display list and
   *        global options.
   * @param parent Qt parent widget (typically the Displays dock content host).
   */
  explicit DisplaysPanel(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);

  /**
   * @brief Full rebuild of Global Options, status, and all Display rows.
   *
   * Prefer @ref refreshStatus() for high-rate status-only updates.
   *
   * @see populateTree()
   */
  void refresh();

  /**
   * @brief Update channel delegate + status icons without rebuilding the tree.
   *
   * Lightweight path for periodic ticks when display structure is unchanged.
   *
   * @see updateGlobalStatus()
   * @see updateChannelDelegate()
   */
  void refreshStatus();

  /**
   * @brief Pause high-rate UI sync (TF pose fields) while the app is in the
   *        background.
   *
   * @param paused @c true to stop the TF pose timer; @c false to resume when
   *        an expanded TfDisplay still needs live pose text.
   * @see updateTfPoseTimerState()
   */
  void setLiveUpdatesPaused(bool paused);

 signals:
  /**
   * @brief Emitted when the user edits Global Fixed Frame.
   *
   * @param frame New fixed-frame name.
   */
  void fixedFrameChanged(const QString& frame);

  /**
   * @brief Emitted when the user edits the viewport background color.
   *
   * @param color New background color.
   */
  void backgroundColorChanged(const QColor& color);

  /**
   * @brief Emitted when the display list or property values change.
   *
   * Listeners should persist session config / request a viewport redraw.
   */
  void displaysChanged();

 protected:
  /**
   * @brief Paints the frosted glass panel chrome behind child widgets.
   *
   * @param event Paint event (rectangle unused; paints @c rect()).
   */
  void paintEvent(QPaintEvent* event) override;

 private slots:
  /**
   * @brief Property / checkbox cell edited: applies the value to manager or
   *        Display properties.
   *
   * Suppressed while @c updating_ is set.
   *
   * @param item Changed tree item.
   * @param column Column index (only @ref kDisplayTreeColValue is applied for
   *        most kinds; check state may apply on the name column).
   */
  void onDisplayItemChanged(QTreeWidgetItem* item, int column);

  /**
   * @brief Add button: opens @ref AddDisplayDialog and inserts a new Display.
   */
  void onAddDisplay();

  /**
   * @brief Duplicate button: clones the selected Display with a unique name.
   */
  void onDuplicateDisplay();

  /**
   * @brief Rename button: prompts for a new instance name.
   */
  void onRenameDisplay();

  /**
   * @brief Remove button: deletes the selected Display from the manager.
   */
  void onRemoveDisplay();

  /**
   * @brief Enables/disables Duplicate / Rename / Remove based on selection.
   */
  void onDisplaySelectionChanged();

  /**
   * @brief TF pose timer tick: refreshes expanded frame pose fields only.
   *
   * @see syncTfExpandedPoseFields()
   */
  void onTfPoseTick();

 private:
  /**
   * @brief Builds splitter, tree, help browser, footer buttons, and wiring.
   */
  void setupUi();

  /**
   * @brief Clears and rebuilds Global Options, status, and all Display rows.
   */
  void populateTree();

  /**
   * @brief Creates the Global Options root and its Fixed Frame / color / FPS
   *        children.
   */
  void populateGlobalOptions();

  /**
   * @brief Creates or refreshes the Global Status group and issue children.
   */
  void populateGlobalStatus();

  /**
   * @brief Appends one tree row (and properties) per Display in the manager.
   */
  void populateDisplays();

  /**
   * @brief Creates property / status / channel children under a Display row.
   *
   * @param display_item Parent tree item for this Display.
   * @param display Non-owning Display pointer from the manager.
   * @param index Index into the manager display list.
   * @param child_index Group child index, or @c -1 for a top-level Display.
   */
  void populateDisplayProperties(QTreeWidgetItem* display_item,
                                 display::Display* display,
                                 std::size_t index, int child_index = -1);

  /**
   * @brief Rebuilds TF-specific frame enable / pose rows under a TfDisplay.
   *
   * @param display_item Parent Display row.
   * @param tf Non-owning TfDisplay pointer.
   */
  void syncTfDisplayProperties(QTreeWidgetItem* display_item,
                               display::TfDisplay* tf);

  /**
   * @brief Lightweight: refresh pose texts for expanded Frames only (~20Hz).
   */
  void syncTfExpandedPoseFields();

  /**
   * @brief Starts/stops @c tf_pose_timer_ based on pause state and whether any
   *        TfDisplay Frames group is expanded.
   */
  void updateTfPoseTimerState();

  /**
   * @brief Refreshes Global Status icons and issue rows (fingerprint-gated).
   */
  void updateGlobalStatus();

  /**
   * @brief Pushes Autolink channel names into @ref DisplayTreeDelegate.
   */
  void updateChannelDelegate();

  /**
   * @brief Updates the help browser from the selected item's description role.
   *
   * @param item Current selection, or @c nullptr to clear.
   */
  void updateHelp(QTreeWidgetItem* item);

  /**
   * @brief Enables Duplicate/Rename/Remove only when a Display row is current.
   */
  void updateActionButtons();

  /**
   * @brief Allocates an unused display instance name with a numeric suffix.
   *
   * @param base Desired base name (e.g. type name).
   * @return Unique name not present in the manager.
   */
  std::string uniqueDisplayName(const std::string& base) const;

  /**
   * @brief Depth-first search for a tree item matching kind and optional keys.
   *
   * @param kind Item discriminator to find.
   * @param display_index Optional display index filter (−1 = any).
   * @param property_key Optional property key filter (empty = any).
   * @return Matching item, or @c nullptr if absent.
   */
  QTreeWidgetItem* findItemByKind(DisplayTreeItemKind kind,
                                  int display_index = -1,
                                  const QString& property_key = {}) const;

  /**
   * @brief Parses @p value and writes it to the Display / global option for
   *        @p item.
   *
   * @param item Tree item that was edited.
   * @param value New value string from the value column.
   */
  void applyPropertyValue(QTreeWidgetItem* item, const QString& value);

  /** Shared manager owning displays and global options. */
  std::shared_ptr<common::VisualizationManager> manager_;

  /** Vertical splitter between the property tree and help browser. */
  QSplitter* splitter_ = nullptr;

  /** Drag-and-drop Displays property tree. */
  DisplayTreeWidget* tree_ = nullptr;

  /** Help / description browser for the selected row. */
  QTextBrowser* help_ = nullptr;

  /** Item delegate for the value column (channel combo, color, etc.). */
  DisplayTreeDelegate* value_delegate_ = nullptr;

  /** Footer: duplicate selected Display. */
  QPushButton* duplicate_button_ = nullptr;

  /** Footer: remove selected Display. */
  QPushButton* remove_button_ = nullptr;

  /** Footer: rename selected Display. */
  QPushButton* rename_button_ = nullptr;

  /** ~20Hz timer for expanded TF pose field text. */
  QTimer* tf_pose_timer_ = nullptr;

  /** When true, TF pose timer stays stopped (app backgrounded). */
  bool live_updates_paused_ = false;

  /**
   * @brief Re-entrancy guard while programmatically updating widgets.
   *
   * Prevents @ref onDisplayItemChanged from writing back into the manager
   * during @ref populateTree() / refresh.
   */
  bool updating_ = false;

  /** Previous error+warn count; used to auto-expand only when issues increase. */
  int global_status_issue_count_ = 0;

  /** Fingerprint of channel names; skip channel-option rebuilds when unchanged. */
  QString channel_list_fingerprint_;

  /** Fingerprint of Global Status issue rows; skip child rebuild when unchanged. */
  QString global_status_fingerprint_;
};

}  // namespace autoviz
