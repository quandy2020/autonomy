/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/view_tree_delegate.hpp"

#include <QComboBox>
#include <QLineEdit>

#include "autoviz/ui/views_panel.hpp"

namespace autoviz {

ViewTreeDelegate::ViewTreeDelegate(QObject* parent)
    : QStyledItemDelegate(parent) {}

void ViewTreeDelegate::setFrameNames(const QStringList& frames) {
  frame_names_ = frames;
}

QWidget* ViewTreeDelegate::createEditor(QWidget* parent,
                                        const QStyleOptionViewItem& option,
                                        const QModelIndex& index) const {
  if (index.column() != kViewTreeColValue) {
    return QStyledItemDelegate::createEditor(parent, option, index);
  }

  const auto kind = static_cast<ViewTreeItemKind>(
      index.data(kViewTreeRoleKind).toInt());
  if (kind != ViewTreeItemKind::kTargetFrame) {
    return QStyledItemDelegate::createEditor(parent, option, index);
  }

  auto* combo = new QComboBox(parent);
  combo->setEditable(true);
  combo->addItems(frame_names_);
  combo->setCurrentText(index.data(Qt::EditRole).toString());
  return combo;
}

void ViewTreeDelegate::setEditorData(QWidget* editor,
                                     const QModelIndex& index) const {
  if (index.column() != kViewTreeColValue) {
    QStyledItemDelegate::setEditorData(editor, index);
    return;
  }

  const QString current = index.data(Qt::EditRole).toString();
  if (auto* combo = qobject_cast<QComboBox*>(editor)) {
    combo->setCurrentText(current);
    return;
  }
  QStyledItemDelegate::setEditorData(editor, index);
}

void ViewTreeDelegate::setModelData(QWidget* editor, QAbstractItemModel* model,
                                    const QModelIndex& index) const {
  if (index.column() != kViewTreeColValue) {
    QStyledItemDelegate::setModelData(editor, model, index);
    return;
  }

  QString value;
  if (auto* combo = qobject_cast<QComboBox*>(editor)) {
    value = combo->currentText().trimmed();
  } else if (auto* line = qobject_cast<QLineEdit*>(editor)) {
    value = line->text().trimmed();
  } else {
    QStyledItemDelegate::setModelData(editor, model, index);
    return;
  }
  model->setData(index, value, Qt::EditRole);
}

void ViewTreeDelegate::updateEditorGeometry(QWidget* editor,
                                            const QStyleOptionViewItem& option,
                                            const QModelIndex& index) const {
  if (index.column() == kViewTreeColValue) {
    editor->setGeometry(option.rect);
    return;
  }
  QStyledItemDelegate::updateEditorGeometry(editor, option, index);
}

}  // namespace autoviz
