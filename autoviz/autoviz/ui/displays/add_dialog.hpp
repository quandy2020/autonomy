/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file add_dialog.hpp
 * @brief Modal dialog for adding a Display by type or by Autolink channel.
 *
 * Hosted from @ref DisplaysPanel::onAddDisplay(). The user picks either a
 * catalogued display type (By Type tab) or a live channel (By Channel tab),
 * assigns a unique instance name, then accepts — callers read the result via
 * @ref selectedType() / @ref selectedName() / @ref selectedChannel().
 *
 * @see DisplaysPanel
 * @see common::DisplayCatalog
 * @see common::VisualizationManager
 */

#pragma once

#include <memory>
#include <vector>

#include <QDialog>
#include <QPoint>
#include <QString>

class QCheckBox;
class QDialogButtonBox;
class QFrame;
class QLineEdit;
class QMouseEvent;
class QPaintEvent;
class QPushButton;
class QStackedWidget;
class QTreeWidget;
class QWidget;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @struct AddDisplaySelection
 * @brief Result of the Add Display dialog (type / instance name / channel).
 *
 * Populated as the user navigates either tab; committed to @c selection_ on
 * accept when @ref AddDisplayDialog::isValid() succeeds.
 */
struct AddDisplaySelection {
  QString type;          /**< Catalog display type id (e.g. @c "TF", @c "PointCloud2"). */
  QString display_name;  /**< Unique instance name shown in the Displays tree. */
  QString channel;       /**< Optional Autolink channel; empty when not applicable. */
};

/**
 * @class AddDisplayDialog
 * @brief Frameless, frosted-card dialog for creating a new Display instance.
 *
 * ## Layout
 *
 * @code
 * ┌──────────────────────────────────────────┐
 * │  By Type  │  By Channel                  │  segment tabs
 * ├──────────────────────────────────────────┤
 * │  (tree of types / channels)              │  stacked content
 * ├──────────────────────────────────────────┤
 * │  Name [____________]                     │
 * │                    [Cancel] [OK]         │
 * └──────────────────────────────────────────┘
 * @endcode
 *
 * ## Tabs
 *
 * - **By Type:** browsable catalog from DisplayFactory / DisplayCatalog;
 *   channel may be empty until the user picks a Topic-tab selection elsewhere.
 * - **By Channel:** live channels from ChannelManager; selecting a channel
 *   suggests a compatible display type and default name.
 *
 * ## Window chrome
 *
 * Custom paint (@ref paintEvent) draws the frosted card; mouse events on the
 * title area implement drag-to-move without a native title bar.
 *
 * @note Does not create the Display itself — @ref DisplaysPanel reads the
 *       selection after @c exec() and calls into VisualizationManager.
 *
 * @see AddDisplaySelection
 * @see DisplaysPanel
 */
class AddDisplayDialog : public QDialog {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the dialog and populates both selection trees.
   *
   * @param manager Shared visualization manager (channel list, catalog access).
   * @param disallowed_display_names Existing instance names that must not be
   *        reused; validated in @ref isValid() / @ref onNameChanged().
   * @param parent Qt parent (typically the Displays panel / main window).
   */
  AddDisplayDialog(
      std::shared_ptr<common::VisualizationManager> manager,
      const QStringList& disallowed_display_names,
      QWidget* parent = nullptr);

  /**
   * @brief Preferred dialog size for first show.
   *
   * @return Size hint large enough for the dual-tab tree and name row.
   */
  QSize sizeHint() const override;

  /**
   * @brief Display type id chosen by the user.
   *
   * @return @c selection_.type after a successful accept; empty if cancelled.
   */
  QString selectedType() const { return selection_.type; }

  /**
   * @brief Instance name to assign to the new Display.
   *
   * @return @c selection_.display_name (unique vs. @c disallowed_display_names_).
   */
  QString selectedName() const { return selection_.display_name; }

  /**
   * @brief Autolink channel bound to the new Display, if any.
   *
   * @return @c selection_.channel; may be empty for types that do not need one.
   */
  QString selectedChannel() const { return selection_.channel; }

 public slots:
  /**
   * @brief Validates the current selection and commits @c selection_.
   *
   * Calls @ref isValid(); on failure shows an error via @ref setError() and
   * keeps the dialog open. On success copies the active tab selection into
   * @c selection_ and closes with @c Accepted.
   */
  void accept() override;

 protected:
  /**
   * @brief Paints the frosted rounded card chrome behind child widgets.
   *
   * @param event Paint event (rectangle unused; paints the dialog card).
   */
  void paintEvent(QPaintEvent* event) override;

  /**
   * @brief Starts window drag when pressing in the non-interactive chrome area.
   *
   * @param event Mouse press; may set @c dragging_ / @c drag_offset_.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Moves the frameless dialog while @c dragging_ is true.
   *
   * @param event Mouse move with global position.
   */
  void mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Ends window drag.
   *
   * @param event Mouse release.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

 private slots:
  /**
   * @brief Segment-control tab changed: switches @c stack_ and restores that
   *        tab's last selection into the name field when appropriate.
   *
   * @param index Stack / segment index (@c display_tab_ or @c topic_tab_).
   */
  void onTabChanged(int index);

  /**
   * @brief Name line-edit edited: marks @c user_edited_name_ and revalidates.
   */
  void onNameChanged();

 private:
  /**
   * @brief Enables/disables OK and refreshes error state from @ref isValid().
   */
  void updateUi();

  /**
   * @brief Returns whether the active tab has a type and a unique name.
   *
   * @return @c true when accept may proceed.
   */
  bool isValid();

  /**
   * @brief Shows or clears the inline validation error string.
   *
   * @param error_text Message to display; empty clears the error.
   */
  void setError(const QString& error_text);

  /**
   * @brief Rebuilds the By Channel tree from ChannelManager.
   */
  void refreshTopicTree();

  /**
   * @brief Shows/hides channel rows based on @c show_unvisualizable_topics_.
   */
  void applyTopicVisibility();

  /**
   * @brief Programmatically selects a segment tab and updates the stack.
   *
   * @param index Tab index to activate.
   */
  void setActiveTab(int index);

  /** Shared manager for catalog and live channel enumeration. */
  std::shared_ptr<common::VisualizationManager> manager_;

  /** Instance names already in use; rejected by @ref isValid(). */
  QStringList disallowed_display_names_;

  /** Committed selection returned by the public getters after accept. */
  AddDisplaySelection selection_;

  /** Last selection made on the By Type tab (preserved across tab switches). */
  AddDisplaySelection display_tab_selection_;

  /** Last selection made on the By Channel tab. */
  AddDisplaySelection topic_tab_selection_;

  /** Segment-control track hosting the two tab buttons. */
  QFrame* segment_track_ = nullptr;

  /** Segment button: browse by display type. */
  QPushButton* tab_by_type_ = nullptr;

  /** Segment button: browse by Autolink channel. */
  QPushButton* tab_by_channel_ = nullptr;

  /** Stacked widget switching between type and channel trees. */
  QStackedWidget* stack_ = nullptr;

  /** Stack index of the By Type page. */
  int display_tab_ = 0;

  /** Stack index of the By Channel page. */
  int topic_tab_ = 1;

  /** Catalog tree (By Type). */
  QTreeWidget* display_tree_ = nullptr;

  /** Channel tree (By Channel). */
  QTreeWidget* topic_tree_ = nullptr;

  /** When checked, shows channels that have no known visualizer. */
  QCheckBox* show_unvisualizable_topics_ = nullptr;

  /** Editable instance name for the new Display. */
  QLineEdit* name_editor_ = nullptr;

  /** Cancel / OK button box. */
  QDialogButtonBox* button_box_ = nullptr;

  /**
   * @brief Key of the last auto-suggested name source (type or channel).
   *
   * Used to overwrite the name field only while the user has not manually
   * edited it (@c user_edited_name_ == false).
   */
  QString last_selection_key_;

  /** When true, selection changes no longer overwrite the name editor. */
  bool user_edited_name_ = false;

  /** True while dragging the frameless window. */
  bool dragging_ = false;

  /** Offset from dialog top-left to the press point during drag. */
  QPoint drag_offset_;
};

}  // namespace autoviz
