/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tree_delegate.hpp
 * @brief Item kinds, roles, and delegates for the Displays property tree.
 *
 * Discriminators and Qt roles are written by @ref DisplaysPanel onto each
 * @c QTreeWidgetItem; @ref DisplayTreeDelegate / @ref DisplayNameTreeDelegate
 * read them to choose editors (channel combo, color picker, checkbox) and
 * paint status icons in the name column.
 *
 * @see DisplaysPanel
 * @see DisplayTreeWidget
 * @see ViewTreeDelegate
 */

#pragma once

#include <QStyledItemDelegate>

namespace autoviz {

/**
 * @enum DisplayTreeItemKind
 * @brief Discriminator stored on each Displays-tree item via
 *        @ref kDisplayTreeRoleKind.
 *
 * Drives:
 * - which editor @ref DisplayTreeDelegate creates;
 * - which manager field @ref DisplaysPanel::onDisplayItemChanged writes;
 * - drag/drop eligibility in @ref DisplayTreeWidget;
 * - help text lookup via @ref kDisplayTreeRoleDescription.
 */
enum class DisplayTreeItemKind {
  kGlobalOptions = 0,       /**< Root group: Fixed Frame, background, FPS. */
  kGlobalFixedFrame,        /**< Global Fixed Frame (editable combo / text). */
  kGlobalBackgroundColor,   /**< Viewport background color (color editor). */
  kGlobalFrameRate,         /**< Target frame rate (float). */
  kGlobalStatus,            /**< Aggregated error/warn status group. */
  kGlobalStatusMessage,     /**< One status line under Global Status. */
  kDisplay,                 /**< Top-level Display instance row. */
  kDisplayChild,            /**< Grouped child Display under a parent Display. */
  kDisplayStatus,           /**< Per-display status summary row. */
  kDisplayChannel,          /**< Bound Autolink channel (combo editor). */
  kDisplayProperty,         /**< Leaf property from the Display property tree. */
  kDisplayPropertyCategory, /**< Non-editable category / group under a Display. */
  kDisplayFrameEnabled,     /**< TF frame enable checkbox under TfDisplay. */
  kDisplayFrameField,       /**< TF frame pose / metadata field (read-mostly). */
  kDisplayTreeNode,         /**< Intermediate tree node (e.g. Frames group). */
  kDisplayAllFramesEnabled, /**< "All Enabled" master checkbox for TF frames. */
};

/**
 * @brief Tree column index for the property / display name (left).
 * @see kDisplayTreeColValue
 */
constexpr int kDisplayTreeColName = 0;

/**
 * @brief Tree column index for the editable / display value (right).
 *
 * Only this column uses @ref DisplayTreeDelegate as its item delegate.
 */
constexpr int kDisplayTreeColValue = 1;

/**
 * @brief Qt::ItemDataRole storing a @ref DisplayTreeItemKind as @c int.
 *
 * Written on column @ref kDisplayTreeColName (and read from the same column by
 * delegates via @c QModelIndex::data).
 */
constexpr int kDisplayTreeRoleKind = Qt::UserRole;

/**
 * @brief Qt::ItemDataRole storing the index into VisualizationManager's
 *        display list (−1 for global / non-display rows).
 */
constexpr int kDisplayTreeRoleDisplayIndex = Qt::UserRole + 1;

/**
 * @brief Qt::ItemDataRole storing the property key string for
 *        @ref DisplayTreeItemKind::kDisplayProperty rows.
 */
constexpr int kDisplayTreeRolePropertyKey = Qt::UserRole + 2;

/**
 * @brief Qt::ItemDataRole storing help / description text shown in the
 *        Displays panel help browser.
 */
constexpr int kDisplayTreeRoleDescription = Qt::UserRole + 3;

/**
 * @brief Qt::ItemDataRole storing the child index within a Display group
 *        for @ref DisplayTreeItemKind::kDisplayChild rows (−1 otherwise).
 */
constexpr int kDisplayTreeRoleChildIndex = Qt::UserRole + 4;

/**
 * @brief Qt::ItemDataRole caching the pre-drag background brush so
 *        @ref DisplayTreeWidget can restore it after highlight.
 */
constexpr int kDisplayTreeRoleDragBgSaved = Qt::UserRole + 5;

/**
 * @brief Qt::ItemDataRole storing the property-factory kind (int) used by
 *        color / enum editors in @ref DisplayTreeDelegate.
 */
constexpr int kDisplayTreeRolePropertyKind = Qt::UserRole + 12;

/**
 * @brief Qt::ItemDataRole storing newline-joined channel names allowed for a
 *        Display Channel editor.
 *
 * When set, the combo is restricted to this list; otherwise the delegate uses
 * the global list from @ref DisplayTreeDelegate::setChannels().
 */
constexpr int kDisplayTreeRoleChannelOptions = Qt::UserRole + 13;

/**
 * @brief Returns whether the item should use a color-picker editor.
 *
 * @param kind Tree item kind.
 * @param property_key Property key (e.g. background color).
 * @param property_kind Optional property-factory kind from
 *        @ref kDisplayTreeRolePropertyKind.
 * @return @c true when @ref DisplayTreeDelegate should open a color editor.
 */
bool IsColorTreeItem(DisplayTreeItemKind kind, const QString& property_key,
                     int property_kind = 0);

/**
 * @brief Maps a value-column index to the sibling name-column index.
 *
 * Delegates store metadata on the name column; this helper obtains that index
 * from any column of the same row.
 *
 * @param index Any column of a Displays-tree row.
 * @return Model index for @ref kDisplayTreeColName on the same row.
 */
QModelIndex DisplayTreeNameColumnIndex(const QModelIndex& index);

/**
 * @class DisplayNameTreeDelegate
 * @brief Custom paint for the Displays-tree name column (status icons, etc.).
 *
 * Draws status glyphs and selection chrome for Display rows; value editing is
 * handled exclusively by @ref DisplayTreeDelegate on column
 * @ref kDisplayTreeColValue.
 *
 * @see DisplayTreeDelegate
 * @see DisplaysPanel
 */
class DisplayNameTreeDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  /**
   * @brief Constructs a name-column delegate.
   *
   * @param parent Typically the @c QTreeWidget that owns the column delegate.
   */
  explicit DisplayNameTreeDelegate(QObject* parent = nullptr);

  /**
   * @brief Paints the name cell including status icon overlays.
   *
   * @param painter Target painter.
   * @param option Style option for the cell.
   * @param index Model index (name column).
   */
  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override;

  /**
   * @brief Preferred size for the name cell (accounts for icon padding).
   *
   * @param option Style option.
   * @param index Model index.
   * @return Size hint for the cell.
   */
  QSize sizeHint(const QStyleOptionViewItem& option,
                 const QModelIndex& index) const override;
};

/**
 * @class DisplayTreeDelegate
 * @brief Custom editor factory for the Displays-tree value column.
 *
 * ## Editor selection
 *
 * | Condition | Editor |
 * |-----------|--------|
 * | Column ≠ @ref kDisplayTreeColValue | Base-class default |
 * | Channel kind / role options | Editable @c QComboBox |
 * | Color property (@ref IsColorTreeItem) | Color button / dialog |
 * | Checkable kinds | Checkbox via @ref editorEvent |
 * | Otherwise | Base-class @c QLineEdit |
 *
 * Kind and property metadata are read from @ref kDisplayTreeRoleKind and
 * related roles on the name column (via @ref DisplayTreeNameColumnIndex).
 *
 * @note Channel list is owned by the panel — call @ref setChannels() when the
 *       Autolink channel set changes.
 *
 * @see DisplaysPanel::updateChannelDelegate()
 * @see DisplayNameTreeDelegate
 */
class DisplayTreeDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  /**
   * @brief Constructs a value-column delegate with an empty channel list.
   *
   * @param parent Typically the @c QTreeWidget that owns the column delegate.
   */
  explicit DisplayTreeDelegate(QObject* parent = nullptr);

  /**
   * @brief Replaces the Autolink channel names offered by channel combos.
   *
   * @param channels Display strings for @c QComboBox::addItems().
   * @see DisplaysPanel::updateChannelDelegate()
   */
  void setChannels(const QStringList& channels);

  /**
   * @brief Creates a cell editor for the given index.
   *
   * @param parent Parent for the editor widget.
   * @param option Style option.
   * @param index Model index being edited.
   * @return New editor widget; ownership transfers to the view.
   */
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;

  /**
   * @brief Custom paint for value cells (e.g. color swatches).
   *
   * @param painter Target painter.
   * @param option Style option for the cell.
   * @param index Model index (value column).
   */
  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override;

  /**
   * @brief Loads the current model value into the editor.
   *
   * @param editor Widget returned by @ref createEditor().
   * @param index Model index being edited.
   */
  void setEditorData(QWidget* editor, const QModelIndex& index) const override;

  /**
   * @brief Commits the editor contents back to the model.
   *
   * DisplaysPanel's @c itemChanged handler then applies the string to the
   * Display / global options.
   *
   * @param editor Active editor widget.
   * @param model Item model to update.
   * @param index Model index being edited.
   */
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;

  /**
   * @brief Positions the editor over the cell (@c option.rect).
   *
   * @param editor Editor widget.
   * @param option Provides @c rect for the cell.
   * @param index Model index.
   */
  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index) const override;

  /**
   * @brief Handles in-place checkbox toggles without opening a line editor.
   *
   * @param event Input event (mouse / key).
   * @param model Item model to update.
   * @param option Style option.
   * @param index Model index under the event.
   * @return @c true if the event was handled.
   */
  bool editorEvent(QEvent* event, QAbstractItemModel* model,
                   const QStyleOptionViewItem& option,
                   const QModelIndex& index) override;

 private:
  /**
   * @brief Cached Autolink channel names for channel-combo editors.
   *
   * Empty until the first @ref setChannels() call; channel editing still works
   * (editable combo) but the drop-down list will be empty.
   */
  QStringList channels_;
};

}  // namespace autoviz
