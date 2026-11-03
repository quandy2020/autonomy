/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractItemModel>
#include <QHash>
#include <QPair>
#include <QStyledItemDelegate>
#include <QTreeView>
#include <QVariant>
#include <QVector>
#include <memory>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

namespace autoviz::widget {

enum class PropertyType {
  kGroup,
  kString,
  kBoolean,
  kInteger,
  kDouble,
  kColor,
};

struct PropertySpec {
  QString id;
  QString name;
  QString description;
  PropertyType type = PropertyType::kString;
  QVariant value;
  QVariant minimum;
  QVariant maximum;
  QVariant step;
  bool read_only = false;
  QVector<PropertySpec> children;
};

class PropertyModel final : public QAbstractItemModel {
  Q_OBJECT

 public:
  enum DataRole {
    kIdRole = Qt::UserRole + 1,
    kTypeRole,
    kMinimumRole,
    kMaximumRole,
    kStepRole,
    kReadOnlyRole,
  };

  explicit PropertyModel(QObject* parent = nullptr);
  ~PropertyModel() override;

  QModelIndex index(int row, int column,
                    const QModelIndex& parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex& child) const override;
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index,
                int role = Qt::DisplayRole) const override;
  bool setData(const QModelIndex& index, const QVariant& value,
               int role = Qt::EditRole) override;
  Qt::ItemFlags flags(const QModelIndex& index) const override;
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role = Qt::DisplayRole) const override;

  void SetProperties(QVector<PropertySpec> properties);
  QVariant Value(const QString& id) const;
  bool SetValue(const QString& id, const QVariant& value);
  void SetValues(const QHash<QString, QVariant>& values);

 signals:
  void ValueChanged(const QString& id, const QVariant& value);
  void ValidationFailed(const QString& id, const QString& message);

 private:
  struct Node;

  Node* NodeForIndex(const QModelIndex& index) const;
  QModelIndex IndexForNode(const Node* node, int column = 1) const;
  void AppendNodes(Node* parent, QVector<PropertySpec> specs,
                   QVector<QPair<QString, QString>>* errors);
  bool NormalizeValue(const Node& node, const QVariant& input, QVariant* output,
                      QString* error) const;
  bool SetNodeValue(Node* node, const QVariant& value, bool emit_value_signal);

  std::unique_ptr<Node> root_;
  QHash<QString, Node*> nodes_by_id_;
};

class PropertyDelegate final : public QStyledItemDelegate {
  Q_OBJECT

 public:
  explicit PropertyDelegate(QObject* parent = nullptr);

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;
  void setEditorData(QWidget* editor, const QModelIndex& index) const override;
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;
  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index) const override;
};

class PropertyEditor final : public QTreeView {
  Q_OBJECT

 public:
  explicit PropertyEditor(QWidget* parent = nullptr);

  PropertyModel* property_model() const noexcept { return model_; }
  void SetProperties(QVector<PropertySpec> properties);
  void SetTone(style::Tone tone);

 signals:
  void PropertyValueChanged(const QString& id, const QVariant& value);
  void PropertyValidationFailed(const QString& id, const QString& message);

 private:
  PropertyModel* model_ = nullptr;
  PropertyDelegate* delegate_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
