/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QStyledItemDelegate>

namespace autoviz {

class ViewTreeDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  explicit ViewTreeDelegate(QObject* parent = nullptr);

  void setFrameNames(const QStringList& frames);

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;
  void setEditorData(QWidget* editor, const QModelIndex& index) const override;
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;
  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index) const override;

 private:
  QStringList frame_names_;
};

}  // namespace autoviz
