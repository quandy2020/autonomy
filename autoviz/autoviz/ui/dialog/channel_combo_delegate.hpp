/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_combo_delegate.hpp
 * @brief QComboBox item delegate for picking an Autolink channel in dialogs
 *        and settings tables.
 *
 * Similar in spirit to the Displays-tree channel editor
 * (@ref DisplayTreeDelegate) but standalone for use in QTableView / form
 * rows outside the Displays panel.
 *
 * @see DisplayTreeDelegate
 * @see ImportRecordDialog
 */

#pragma once

#include <QStyledItemDelegate>

namespace autoviz {

/**
 * @class ChannelComboDelegate
 * @brief Creates an editable QComboBox populated with @c channels_ for the
 *        cell editor.
 *
 * ## Lifecycle
 *
 * 1. Caller invokes @ref setChannels() whenever the live channel set changes.
 * 2. The view calls @ref createEditor() → @ref setEditorData().
 * 3. On commit, @ref setModelData() writes the combo's current text to
 *    @c Qt::EditRole.
 *
 * @note The combo is editable so users can type a channel that is not yet in
 *       the cached list (e.g. about to be published).
 */
class ChannelComboDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  /**
   * @brief Constructs a delegate with an empty channel list.
   *
   * @param parent Typically the view that owns the column delegate.
   */
  explicit ChannelComboDelegate(QObject* parent = nullptr);

  /**
   * @brief Replaces the channel names offered by the combo editor.
   *
   * @param channels Display strings for @c QComboBox::addItems().
   */
  void setChannels(const QStringList& channels);

  /**
   * @brief Creates an editable QComboBox filled with @c channels_.
   *
   * @param parent Parent for the editor widget.
   * @param option Style option (unused beyond defaults).
   * @param index Model index being edited.
   * @return New combo editor; ownership transfers to the view.
   */
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;

  /**
   * @brief Sets the combo's current text from @c Qt::EditRole.
   *
   * @param editor Widget returned by @ref createEditor().
   * @param index Model index being edited.
   */
  void setEditorData(QWidget* editor, const QModelIndex& index) const override;

  /**
   * @brief Commits the combo's current text back to the model.
   *
   * @param editor Active combo editor.
   * @param model Item model to update.
   * @param index Model index being edited.
   */
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;

 private:
  /**
   * @brief Cached Autolink channel names for the combo drop-down.
   *
   * Empty until the first @ref setChannels() call; editing still works but
   * the list will be empty.
   */
  QStringList channels_;
};

}  // namespace autoviz
