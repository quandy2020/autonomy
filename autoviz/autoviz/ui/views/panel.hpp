/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel.hpp
 * @brief Camera Views panel — RViz2-style view type, live properties, and saved
 *        viewpoints.
 *
 * The Views panel is hosted in the right sidebar dock (@c ViewsDock). It binds
 * to the active viewport's @ref rendering::ViewController and optionally to
 * @ref common::VisualizationManager for TF frame names used by Target Frame.
 *
 * @see ViewTreeDelegate
 * @see rendering::ViewController
 * @see common::SavedViewConfig
 */

#pragma once

#include <vector>

#include <QElapsedTimer>
#include <QWidget>

class QComboBox;
class QPaintEvent;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/views/tree_delegate.hpp"

namespace autoviz {
namespace common {
class VisualizationManager;
}
namespace rendering {
class ViewController;
}

/**
 * @enum ViewTreeItemKind
 * @brief Discriminator stored on each QTreeWidgetItem via @ref kViewTreeRoleKind.
 *
 * Drives:
 * - which editor @ref ViewTreeDelegate creates (e.g. combo for Target Frame);
 * - which ViewController field @ref ViewsPanel::onTreeItemChanged writes;
 * - show/hide and label remapping in @ref ViewsPanel::updatePropertyVisibility
 *   (Orbit vs FPS vs TopDownOrtho, aligned with RViz2 property names).
 */
enum class ViewTreeItemKind {
  kCurrentView = 0,       /**< Root group: live camera under the active controller. */
  kNearClip,              /**< Near clip plane distance (float). */
  kInvertZ,               /**< Invert Z axis (checkable). */
  kTargetFrame,           /**< Look-at / follow TF frame (editable combo). */
  kDistance,              /**< Orbit distance, or Ortho scale when remapped. */
  kFocalShapeSize,        /**< Focal marker size (Orbit family only). */
  kFocalShapeFixedSize,   /**< Whether focal shape ignores distance scaling. */
  kYaw,                   /**< Orbit/FPS yaw, or Ortho angle when remapped. */
  kPitch,                 /**< Orbit/FPS pitch (hidden for TopDownOrtho). */
  kFocalPointGroup,       /**< Parent row for X/Y/Z; value shows a summary string. */
  kFocalPointX,           /**< Focal point or FPS eye position X. */
  kFocalPointY,           /**< Focal point or FPS eye position Y. */
  kFocalPointZ,           /**< Focal point or FPS eye position Z (hidden in Ortho). */
  kSavedView,             /**< Top-level row for a bookmark in @c saved_views_. */
};

/**
 * @brief Tree column index for the property name (left).
 * @see kViewTreeColValue
 */
constexpr int kViewTreeColName = 0;

/**
 * @brief Tree column index for the editable / display value (right).
 *
 * Only this column uses @ref ViewTreeDelegate as its item delegate.
 */
constexpr int kViewTreeColValue = 1;

/**
 * @brief Qt::ItemDataRole storing a @ref ViewTreeItemKind as @c int.
 *
 * Written on column @ref kViewTreeColName (and read from the same column by
 * the delegate via @c QModelIndex::data).
 */
constexpr int kViewTreeRoleKind = Qt::UserRole;

/**
 * @brief Qt::ItemDataRole storing the index into @c saved_views_ for
 *        @ref ViewTreeItemKind::kSavedView rows (−1 otherwise).
 */
constexpr int kViewTreeRoleSavedIndex = Qt::UserRole + 1;

/**
 * @class ViewsPanel
 * @brief Sidebar panel for choosing the view controller type, editing live
 *        camera properties, and managing saved viewpoints.
 *
 * ## Layout
 *
 * @code
 * ┌─────────────────────────────────────┐
 * │ Type [Orbit ▾]              [Zero]  │  toolbar
 * ├─────────────────────────────────────┤
 * │ ▾ Current View          Orbit (…)   │
 * │     Near Clip Distance        0.01  │
 * │     Invert Z Axis             ☐     │
 * │     Target Frame          <Fixed>   │
 * │     …                               │
 * │   View 1                  Orbit (…) │  saved bookmarks
 * ├─────────────────────────────────────┤
 * │ [Save] [Remove] [Rename]            │  footer
 * └─────────────────────────────────────┘
 * @endcode
 *
 * ## Data flow
 *
 * - **Out:** edits and Zero/type changes call into @ref rendering::ViewController
 *   and emit @ref viewChanged(); Save/Remove/Rename mutate @c saved_views_ and
 *   emit @ref viewsChanged() so VisualizationFrame can persist session config.
 * - **In:** @ref refreshFromController() / @ref setViewController() rebuild or
 *   refresh the tree from the controller; @ref setSavedViews() replaces bookmarks.
 *
 * ## Styling
 *
 * Draws a frosted card via @c PaintPanelFrostedCard in @ref paintEvent(); tree
 * and footer use theme helpers under @c autoviz/ui/theme/.
 *
 * @note The panel does not own the ViewController or VisualizationManager;
 *       callers (VisualizationFrame / FrameViewport) keep those lifetimes.
 *
 * @see ViewTreeDelegate
 * @see FrameViewport
 * @see common::SessionConfig
 */
class ViewsPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the Views panel and builds the initial property tree.
   *
   * @param view_controller Active viewport camera controller, or @c nullptr
   *        until @ref setViewController() is called (type combo still lists
   *        built-in types; property values stay empty).
   * @param manager Optional visualization manager used for TF frame names in
   *        the Target Frame editor. May be @c nullptr; call @ref setManager()
   *        later when available.
   * @param parent Qt parent widget (typically the Views dock content host).
   */
  explicit ViewsPanel(rendering::ViewController* view_controller,
                      common::VisualizationManager* manager = nullptr,
                      QWidget* parent = nullptr);

  /**
   * @brief Returns the in-memory list of saved viewpoint bookmarks.
   *
   * Used when capturing session config (VisualizationFrame / FrameSession).
   *
   * @return Copy of @c saved_views_.
   * @see setSavedViews()
   * @see common::SavedViewConfig
   */
  std::vector<common::SavedViewConfig> savedViews() const;

  /**
   * @brief Replaces saved viewpoints and rebuilds the tree.
   *
   * @param views Bookmark list from session config (order preserved as
   *        top-level rows under Current View).
   * @see savedViews()
   */
  void setSavedViews(const std::vector<common::SavedViewConfig>& views);

  /**
   * @brief Attaches or replaces the visualization manager.
   *
   * Refreshes the Target Frame combo contents and, when both manager and
   * view controller are set, wires the controller's FrameManager pointer.
   *
   * @param manager Non-owning pointer; may be @c nullptr to clear.
   * @see refreshFrameList()
   */
  void setManager(common::VisualizationManager* manager);

  /**
   * @brief Binds a (possibly new) view controller and rebuilds UI from it.
   *
   * Repopulates the type selector, rebuilds the property tree, and attaches
   * FrameManager when a manager is already set.
   *
   * @param view_controller Non-owning pointer to the active viewport controller.
   * @note Called when the user switches the active 3D panel or render backend
   *       recreates the window.
   */
  void setViewController(rendering::ViewController* view_controller);

  /**
   * @brief Syncs Current View values from the controller.
   *
   * Does not rebuild the whole tree (unlike @ref setViewController()). Type
   * combo is left alone — it is refreshed via @ref setViewController().
   *
   * @see syncCameraValuesFromController()
   * @see updateCurrentViewValues()
   */
  void refreshFromController();

  /**
   * @brief Lightweight camera-pose sync for live Orbit/FPS drag.
   *
   * Updates numeric Current View cells only; skips type combo rebuild and
   * property show/hide (those do not change while dragging).
   *
   * @see refreshFromController()
   */
  void syncCameraValuesFromController();

  /**
   * @brief Refresh TF frame list for the Target Frame combo without rebuilding
   *        the tree.
   *
   * Lightweight path when the TF tree changes but camera pose did not.
   *
   * @see updateFrameDelegate()
   */
  void refreshFrameList();

  /**
   * @brief Resets the camera to the controller default (same as Zero button).
   *
   * Public entry for keyboard shortcut @c Z routed from VisualizationFrame.
   *
   * @see onZeroClicked()
   */
  void zeroView();

 signals:
  /**
   * @brief Emitted when the saved-view list changes (Save / Remove / Rename).
   *
   * Listeners should write @ref savedViews() into session config.
   */
  void viewsChanged();

  /**
   * @brief Emitted when the live camera changes (type, Zero, or property edit).
   *
   * Listeners typically update the manager's view-controller name and request
   * a viewport redraw.
   */
  void viewChanged();

 protected:
  /**
   * @brief Paints the frosted glass panel chrome behind child widgets.
   *
   * @param event Paint event (rectangle unused; paints @c rect()).
   */
  void paintEvent(QPaintEvent* event) override;

 private slots:
  /**
   * @brief Type combo activated: switches ViewController type by stored
   *        item data string (Orbit, XYOrbit, TopDownOrtho, FPS, …).
   *
   * @param index Combo index; ignored while @c updating_ is true.
   */
  void onTypeChanged(int index);

  /**
   * @brief Zero button: @c view_controller_->reset(), then refresh UI and
   *        emit @ref viewChanged().
   */
  void onZeroClicked();

  /**
   * @brief Save button: appends a snapshot of the current ViewState as
   *        "View N" and emits @ref viewsChanged().
   */
  void onSaveClicked();

  /**
   * @brief Remove button: deletes the selected @ref ViewTreeItemKind::kSavedView
   *        row from @c saved_views_.
   */
  void onRemoveClicked();

  /**
   * @brief Rename button: prompts for a new name on the selected saved view.
   */
  void onRenameClicked();

  /**
   * @brief Property cell edited: parses the value column and writes the
   *        corresponding ViewController field.
   *
   * Checkbox kinds (@ref ViewTreeItemKind::kInvertZ,
   * @ref ViewTreeItemKind::kFocalShapeFixedSize) use check state; others use
   * text. Suppressed while @c updating_ is set.
   *
   * @param item Changed tree item.
   * @param column Column index (only @ref kViewTreeColValue is applied).
   */
  void onTreeItemChanged(QTreeWidgetItem* item, int column);

  /**
   * @brief Enables/disables Remove and Rename based on whether a saved view
   *        row is selected.
   */
  void onTreeSelectionChanged();

  /**
   * @brief Activates a saved view (click / Enter): restores ViewState onto the
   *        controller and refreshes Current View fields.
   *
   * @param item Activated item (must be @ref ViewTreeItemKind::kSavedView).
   * @param column Unused (Qt signal signature).
   */
  void onTreeItemActivated(QTreeWidgetItem* item, int column);

 private:
  /**
   * @brief Builds toolbar, property tree, footer buttons, and signal wiring.
   */
  void setupUi();

  /**
   * @brief Fills the Type combo with RViz2-aligned built-in controller names.
   *
   * Maps legacy Autoviz aliases (@c TopDown → @c TopDownOrtho,
   * @c FPSMotion → @c FPS) when selecting the current item.
   */
  void populateTypeSelector();

  /**
   * @brief Clears and rebuilds Current View properties plus all saved-view rows.
   */
  void populateTree();

  /**
   * @brief Creates the editable property children under the Current View root.
   *
   * @param parent The @ref ViewTreeItemKind::kCurrentView top-level item.
   */
  void populateCurrentViewProperties(QTreeWidgetItem* parent);

  /**
   * @brief Copies ViewController state into Current View value cells.
   *
   * Chooses FPS eye pose vs Orbit focal point based on controller type.
   *
   * @param update_visibility When @c true, also runs @ref updatePropertyVisibility().
   */
  void updateCurrentViewValues(bool update_visibility = true);

  /**
   * @brief Shows/hides and relabels properties for Orbit / FPS / Ortho.
   *
   * Label mapping follows RViz2 (e.g. Ortho: Distance→Scale, Yaw→Angle).
   */
  void updatePropertyVisibility();

  /**
   * @brief Enables Remove/Rename only when a saved-view row is current.
   */
  void updateActionButtons();

  /**
   * @brief Pushes TF frame names (plus Fixed sentinel) into @ref ViewTreeDelegate.
   */
  void updateFrameDelegate();

  /**
   * @brief Formats a controller type for display, e.g. @c "Orbit (autoviz)".
   *
   * @param type Raw type id stored in combo item data / SavedViewConfig.
   * @return Localized display string.
   */
  QString formattedTypeName(const QString& type) const;

  /**
   * @brief Depth-first search for the first tree item with the given kind.
   *
   * @param kind Item discriminator to find.
   * @return Matching item, or @c nullptr if absent (e.g. tree not built yet).
   */
  QTreeWidgetItem* findItemByKind(ViewTreeItemKind kind) const;

  /** Non-owning; provides TF frames via FrameManager. */
  common::VisualizationManager* manager_ = nullptr;

  /** Non-owning; live camera for the active viewport. */
  rendering::ViewController* view_controller_ = nullptr;

  /** Item delegate for the value column (Target Frame combo, etc.). */
  ViewTreeDelegate* value_delegate_ = nullptr;

  /** View-controller type dropdown (Orbit, FPS, …). */
  QComboBox* type_selector_ = nullptr;

  /** Two-column property / saved-view tree. */
  QTreeWidget* tree_ = nullptr;

  /** Footer: remove selected bookmark (disabled until a saved row is selected). */
  QPushButton* remove_button_ = nullptr;

  /** Footer: rename selected bookmark. */
  QPushButton* rename_button_ = nullptr;

  /** In-memory bookmarks persisted through session config. */
  std::vector<common::SavedViewConfig> saved_views_;

  /**
   * @brief Re-entrancy guard while programmatically updating widgets.
   *
   * Prevents @ref onTypeChanged / @ref onTreeItemChanged from writing back
   * into the controller during @ref populateTree() / refresh.
   */
  bool updating_ = false;

  /** Throttle for @ref syncCameraValuesFromController during Orbit drag. */
  QElapsedTimer camera_sync_elapsed_;
};

}  // namespace autoviz
