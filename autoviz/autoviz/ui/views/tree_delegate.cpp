/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tree_delegate.cpp
 * @brief Implementation of @ref autoviz::ViewTreeDelegate.
 *
 * Target Frame cells get an editable QComboBox; boolean rows use the tree's
 * native value-column checkbox (no editor). All other value cells fall through
 * to QStyledItemDelegate (typically QLineEdit).
 *
 * @see tree_delegate.hpp
 * @see ViewsPanel
 */

#include "autoviz/ui/views/tree_delegate.hpp"

#include <QComboBox>
#include <QLineEdit>
#include <QPainter>
#include <QStyle>

#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/views/panel.hpp"

namespace autoviz {
namespace {

QModelIndex NameColumnIndex(const QModelIndex& index) {
  if (!index.isValid() || index.column() == kViewTreeColName) {
    return index;
  }
  return index.sibling(index.row(), kViewTreeColName);
}

ViewTreeItemKind KindFromIndex(const QModelIndex& index) {
  return static_cast<ViewTreeItemKind>(
      NameColumnIndex(index).data(kViewTreeRoleKind).toInt());
}

bool IsCheckableKind(ViewTreeItemKind kind) {
  return kind == ViewTreeItemKind::kInvertZ ||
         kind == ViewTreeItemKind::kFocalShapeFixedSize ||
         kind == ViewTreeItemKind::kTargetFrameChoice;
}

}  // namespace

ViewTreeDelegate::ViewTreeDelegate(QObject* parent)
    : QStyledItemDelegate(parent) {}

void ViewTreeDelegate::setFrameNames(const QStringList& frames) {
  frame_names_ = frames;
}

void ViewTreeDelegate::paint(QPainter* painter,
                             const QStyleOptionViewItem& option,
                             const QModelIndex& index) const {
  // Keep value-column checkboxes / text readable: name column owns the
  // selection pill (ContinuousNameColumnDelegate).
  QStyleOptionViewItem cleared = option;
  cleared.state.setFlag(QStyle::State_Selected, false);
  cleared.state.setFlag(QStyle::State_MouseOver, false);
  cleared.backgroundBrush = Qt::NoBrush;
  QStyledItemDelegate::paint(painter, cleared, index);
}

QWidget* ViewTreeDelegate::createEditor(QWidget* parent,
                                        const QStyleOptionViewItem& option,
                                        const QModelIndex& index) const {
  if (index.column() != kViewTreeColValue) {
    return QStyledItemDelegate::createEditor(parent, option, index);
  }

  const ViewTreeItemKind kind = KindFromIndex(index);
  if (IsCheckableKind(kind)) {
    return nullptr;
  }
  if (kind == ViewTreeItemKind::kTargetFrame) {
    auto* combo = new QComboBox(parent);
    combo->setEditable(true);
    combo->setInsertPolicy(QComboBox::NoInsert);
    combo->addItems(frame_names_);
    StyleCompactSettingsField(combo);
    return combo;
  }
  auto* line = qobject_cast<QLineEdit*>(
      QStyledItemDelegate::createEditor(parent, option, index));
  if (line != nullptr) {
    StyleCompactSettingsField(line);
    return line;
  }
  return QStyledItemDelegate::createEditor(parent, option, index);
}

void ViewTreeDelegate::setEditorData(QWidget* editor,
                                     const QModelIndex& index) const {
  if (index.column() != kViewTreeColValue) {
    QStyledItemDelegate::setEditorData(editor, index);
    return;
  }

  if (auto* combo = qobject_cast<QComboBox*>(editor)) {
    const QString text = index.data(Qt::EditRole).toString();
    const int found = combo->findText(text);
    if (found >= 0) {
      combo->setCurrentIndex(found);
    } else {
      combo->setEditText(text);
    }
    return;
  }
  if (auto* line = qobject_cast<QLineEdit*>(editor)) {
    line->setText(index.data(Qt::EditRole).toString());
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

  if (auto* combo = qobject_cast<QComboBox*>(editor)) {
    model->setData(index, combo->currentText().trimmed(), Qt::EditRole);
    return;
  }
  if (auto* line = qobject_cast<QLineEdit*>(editor)) {
    model->setData(index, line->text().trimmed(), Qt::EditRole);
    return;
  }
  QStyledItemDelegate::setModelData(editor, model, index);
}

void ViewTreeDelegate::updateEditorGeometry(QWidget* editor,
                                            const QStyleOptionViewItem& option,
                                            const QModelIndex& index) const {
  if (index.column() == kViewTreeColValue) {
    editor->setGeometry(option.rect.adjusted(0, 1, 0, -1));
    return;
  }
  QStyledItemDelegate::updateEditorGeometry(editor, option, index);
}

}  // namespace autoviz
