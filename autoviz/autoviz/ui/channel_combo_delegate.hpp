/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QStyledItemDelegate>

namespace autoviz {

class ChannelComboDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  explicit ChannelComboDelegate(QObject* parent = nullptr);

  void setChannels(const QStringList& channels);

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;
  void setEditorData(QWidget* editor, const QModelIndex& index) const override;
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;

 private:
  QStringList channels_;
};

}  // namespace autoviz
