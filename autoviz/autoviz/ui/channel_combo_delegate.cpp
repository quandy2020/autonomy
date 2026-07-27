/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channel_combo_delegate.hpp"

#include <QComboBox>

namespace autoviz {

ChannelComboDelegate::ChannelComboDelegate(QObject* parent)
    : QStyledItemDelegate(parent) {}

void ChannelComboDelegate::setChannels(const QStringList& channels) {
  channels_ = channels;
}

QWidget* ChannelComboDelegate::createEditor(QWidget* parent,
                                            const QStyleOptionViewItem& option,
                                            const QModelIndex& index) const {
  if (index.column() != 2) {
    return QStyledItemDelegate::createEditor(parent, option, index);
  }
  auto* combo = new QComboBox(parent);
  combo->setEditable(true);
  combo->addItems(channels_);
  return combo;
}

void ChannelComboDelegate::setEditorData(QWidget* editor,
                                         const QModelIndex& index) const {
  if (index.column() != 2) {
    QStyledItemDelegate::setEditorData(editor, index);
    return;
  }
  auto* combo = qobject_cast<QComboBox*>(editor);
  if (combo == nullptr) {
    return;
  }
  combo->setCurrentText(index.data(Qt::EditRole).toString());
}

void ChannelComboDelegate::setModelData(QWidget* editor,
                                        QAbstractItemModel* model,
                                        const QModelIndex& index) const {
  if (index.column() != 2) {
    QStyledItemDelegate::setModelData(editor, model, index);
    return;
  }
  auto* combo = qobject_cast<QComboBox*>(editor);
  if (combo == nullptr) {
    return;
  }
  model->setData(index, combo->currentText(), Qt::EditRole);
}

}  // namespace autoviz
