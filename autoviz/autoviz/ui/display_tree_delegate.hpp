/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QStyledItemDelegate>

namespace autoviz {

enum class DisplayTreeItemKind {
  kGlobalOptions = 0,
  kGlobalFixedFrame,
  kGlobalBackgroundColor,
  kGlobalShowGrid,
  kGlobalFrameRate,
  kGlobalStatus,
  kGlobalStatusMessage,
  kDisplay,
  kDisplayChild,
  kDisplayStatus,
  kDisplayChannel,
  kDisplayProperty,
  kDisplayPropertyCategory,
  kDisplayFrameEnabled,
  kDisplayFrameField,
  kDisplayTreeNode,
  kDisplayAllFramesEnabled,
};

constexpr int kDisplayTreeColName = 0;
constexpr int kDisplayTreeColValue = 1;

constexpr int kDisplayTreeRoleKind = Qt::UserRole;
constexpr int kDisplayTreeRoleDisplayIndex = Qt::UserRole + 1;
constexpr int kDisplayTreeRolePropertyKey = Qt::UserRole + 2;
constexpr int kDisplayTreeRoleDescription = Qt::UserRole + 3;
constexpr int kDisplayTreeRoleChildIndex = Qt::UserRole + 4;
constexpr int kDisplayTreeRoleDragBgSaved = Qt::UserRole + 5;
constexpr int kDisplayTreeRolePropertyKind = Qt::UserRole + 12;
/** Newline-joined channel names allowed for a Display Channel editor. */
constexpr int kDisplayTreeRoleChannelOptions = Qt::UserRole + 13;

bool IsColorTreeItem(DisplayTreeItemKind kind, const QString& property_key,
                     int property_kind = 0);

QModelIndex DisplayTreeNameColumnIndex(const QModelIndex& index);

class DisplayNameTreeDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  explicit DisplayNameTreeDelegate(QObject* parent = nullptr);

  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override;
  QSize sizeHint(const QStyleOptionViewItem& option,
                 const QModelIndex& index) const override;
};

class DisplayTreeDelegate : public QStyledItemDelegate {
  Q_OBJECT

 public:
  explicit DisplayTreeDelegate(QObject* parent = nullptr);

  void setChannels(const QStringList& channels);

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;
  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override;
  void setEditorData(QWidget* editor, const QModelIndex& index) const override;
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;
  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index) const override;
  bool editorEvent(QEvent* event, QAbstractItemModel* model,
                   const QStyleOptionViewItem& option,
                   const QModelIndex& index) override;

 private:
  QStringList channels_;
};

}  // namespace autoviz
