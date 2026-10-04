/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tree_delegate.hpp
 * @brief Item delegate for the Views panel property-tree value column.
 *
 * Most cells use the default QStyledItemDelegate line editor. The Target Frame
 * property instead opens an editable QComboBox populated with TF frame names
 * (plus the Fixed sentinel) supplied by @ref setFrameNames().
 *
 * @see ViewsPanel
 * @see ViewTreeItemKind
 * @see kViewTreeColValue
 */

#pragma once

#include <QStyledItemDelegate>

namespace autoviz {

/**
 * @class ViewTreeDelegate
 * @brief Custom editor factory for @ref ViewsPanel's value column.
 *
 * ## Editor selection
 *
 * | Condition | Editor |
 * |-----------|--------|
 * | Column ≠ @ref kViewTreeColValue | Base-class default |
 * | Kind ≠ @ref ViewTreeItemKind::kTargetFrame | Base-class QLineEdit |
 * | Kind = Target Frame | Editable @c QComboBox with @c frame_names_ |
 *
 * Kind is read from @ref kViewTreeRoleKind on the model index (same role
 * ViewsPanel writes via @c SetTreeItemMeta on the name column; the model
 * exposes it for the value index as well through the item's data).
 *
 * ## Geometry
 *
 * Value-column editors are forced to @c option.rect so the combo fills the
 * cell; other columns defer to QStyledItemDelegate.
 *
 * @note This class does not own the frame name list source — ViewsPanel calls
 *       @ref setFrameNames() whenever TF frames change.
 */
class ViewTreeDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  /**
   * @brief Constructs a delegate with an empty frame list.
   *
   * @param parent Typically the QTreeWidget that owns the column delegate
   *        (Qt parent for QObject lifetime).
   */
  explicit ViewTreeDelegate(QObject* parent = nullptr);

  /**
   * @brief Replaces the TF frame names offered by the Target Frame combo.
   *
   * Callers usually prepend the Fixed-frame sentinel, then all names from
   * FrameManager, sorted case-insensitively.
   *
   * @param frames Display strings for @c QComboBox::addItems().
   * @see ViewsPanel::updateFrameDelegate()
   */
  void setFrameNames(const QStringList& frames);

  /**
   * @brief Creates a cell editor for the given index.
   *
   * @param parent Parent for the editor widget.
   * @param option Style option (unused for Target Frame path beyond defaults).
   * @param index Model index being edited.
   * @return New editor widget; ownership transfers to the view.
   *
   * @see setEditorData()
   * @see setModelData()
   */
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;

  /**
   * @brief Loads the current model value into the editor.
   *
   * For QComboBox editors, sets @c currentText from @c Qt::EditRole.
   *
   * @param editor Widget returned by @ref createEditor().
   * @param index Model index being edited.
   */
  void setEditorData(QWidget* editor, const QModelIndex& index) const override;

  /**
   * @brief Commits the editor contents back to the model.
   *
   * Trims whitespace from combo or line-edit text, then writes @c Qt::EditRole.
   * ViewsPanel's @c itemChanged handler then applies the string to the
   * ViewController.
   *
   * @param editor Active editor widget.
   * @param model Item model to update.
   * @param index Model index being edited.
   */
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;

  /**
   * @brief Positions the editor over the cell.
   *
   * Value-column editors use the full @c option.rect; other columns use the
   * base-class geometry policy.
   *
   * @param editor Editor widget.
   * @param option Provides @c rect for the cell.
   * @param index Model index (column selects the layout path).
   */
  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index) const override;

 private:
  /**
   * @brief Cached TF frame names for the Target Frame QComboBox.
   *
   * Empty until the first @ref setFrameNames() call; Target Frame editing
   * still works (editable combo) but the drop-down list will be empty.
   */
  QStringList frame_names_;
};

}  // namespace autoviz
