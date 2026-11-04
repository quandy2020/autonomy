/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/property_editor.hpp"

#include <QColor>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFrame>
#include <QHeaderView>
#include <QLineEdit>
#include <QMetaType>
#include <QPointer>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {
namespace {

QVariant DefaultPropertyValue(const PropertySpec& spec) {
  switch (spec.type) {
    case PropertyType::kString:
      return QString();
    case PropertyType::kBoolean:
      return false;
    case PropertyType::kInteger:
      if (spec.minimum.isValid()) {
        return spec.minimum.toLongLong();
      }
      if (spec.maximum.isValid() && spec.maximum.toLongLong() < 0) {
        return spec.maximum.toLongLong();
      }
      return QVariant::fromValue<qlonglong>(0);
    case PropertyType::kDouble:
      if (spec.minimum.isValid()) {
        return spec.minimum.toDouble();
      }
      if (spec.maximum.isValid() && spec.maximum.toDouble() < 0.0) {
        return spec.maximum.toDouble();
      }
      return 0.0;
    case PropertyType::kColor:
      return QColor(Qt::transparent);
    case PropertyType::kGroup:
      return {};
  }
  return {};
}

}  // namespace

struct PropertyModel::Node {
  PropertySpec spec;
  Node* parent = nullptr;
  std::vector<std::unique_ptr<Node>> children;

  int Row() const {
    if (parent == nullptr) {
      return 0;
    }
    for (int row = 0; row < static_cast<int>(parent->children.size()); ++row) {
      if (parent->children[static_cast<std::size_t>(row)].get() == this) {
        return row;
      }
    }
    return -1;
  }
};

PropertyModel::PropertyModel(QObject* parent)
    : QAbstractItemModel(parent), root_(std::make_unique<Node>()) {
  root_->spec.type = PropertyType::kGroup;
}

PropertyModel::~PropertyModel() = default;

QModelIndex PropertyModel::index(int row, int column,
                                 const QModelIndex& parent) const {
  if (row < 0 || column < 0 || column >= columnCount(parent)) {
    return {};
  }
  Node* parent_node = NodeForIndex(parent);
  if (row >= static_cast<int>(parent_node->children.size())) {
    return {};
  }
  return createIndex(
      row, column, parent_node->children[static_cast<std::size_t>(row)].get());
}

QModelIndex PropertyModel::parent(const QModelIndex& child) const {
  if (!child.isValid()) {
    return {};
  }
  const Node* node = NodeForIndex(child);
  const Node* parent_node = node->parent;
  if (parent_node == nullptr || parent_node == root_.get()) {
    return {};
  }
  return createIndex(parent_node->Row(), 0, const_cast<Node*>(parent_node));
}

int PropertyModel::rowCount(const QModelIndex& parent) const {
  if (parent.isValid() && parent.column() != 0) {
    return 0;
  }
  return static_cast<int>(NodeForIndex(parent)->children.size());
}

int PropertyModel::columnCount(const QModelIndex& parent) const {
  Q_UNUSED(parent);
  return 2;
}

QVariant PropertyModel::data(const QModelIndex& index, int role) const {
  if (!index.isValid()) {
    return {};
  }
  const Node* node = NodeForIndex(index);
  const PropertySpec& spec = node->spec;
  switch (role) {
    case Qt::DisplayRole:
      if (index.column() == 0) {
        return spec.name;
      }
      if (spec.type == PropertyType::kGroup) {
        return {};
      }
      if (spec.type == PropertyType::kColor) {
        const QColor color = spec.value.value<QColor>();
        return color.isValid() ? color.name(QColor::HexArgb) : QString();
      }
      return spec.value;
    case Qt::EditRole:
      return index.column() == 1 ? spec.value : QVariant();
    case Qt::ToolTipRole:
      return spec.description;
    case Qt::CheckStateRole:
      if (index.column() == 1 && spec.type == PropertyType::kBoolean) {
        return spec.value.toBool() ? Qt::Checked : Qt::Unchecked;
      }
      return {};
    case Qt::FontRole:
      if (spec.type == PropertyType::kGroup) {
        QFont font;
        font.setBold(true);
        return font;
      }
      return {};
    case Qt::DecorationRole:
      if (index.column() == 1 && spec.type == PropertyType::kColor) {
        return spec.value.value<QColor>();
      }
      return {};
    case kIdRole:
      return spec.id;
    case kTypeRole:
      return static_cast<int>(spec.type);
    case kMinimumRole:
      return spec.minimum;
    case kMaximumRole:
      return spec.maximum;
    case kStepRole:
      return spec.step;
    case kReadOnlyRole:
      return spec.read_only;
    default:
      return {};
  }
}

bool PropertyModel::setData(const QModelIndex& index, const QVariant& value,
                            int role) {
  if (!index.isValid() || index.column() != 1) {
    return false;
  }
  Node* node = NodeForIndex(index);
  if (node->spec.read_only || node->spec.type == PropertyType::kGroup) {
    return false;
  }
  QVariant candidate = value;
  if (role == Qt::CheckStateRole && node->spec.type == PropertyType::kBoolean) {
    candidate = value.toInt() == Qt::Checked;
  } else if (role != Qt::EditRole) {
    return false;
  }
  return SetNodeValue(node, candidate, true);
}

Qt::ItemFlags PropertyModel::flags(const QModelIndex& index) const {
  if (!index.isValid()) {
    return Qt::NoItemFlags;
  }
  const Node* node = NodeForIndex(index);
  Qt::ItemFlags result = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  if (index.column() != 1 || node->spec.read_only ||
      node->spec.type == PropertyType::kGroup) {
    return result;
  }
  if (node->spec.type == PropertyType::kBoolean) {
    result |= Qt::ItemIsUserCheckable;
  } else {
    result |= Qt::ItemIsEditable;
  }
  return result;
}

QVariant PropertyModel::headerData(int section, Qt::Orientation orientation,
                                   int role) const {
  if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
    return QAbstractItemModel::headerData(section, orientation, role);
  }
  return section == 0 ? tr("Property") : tr("Value");
}

void PropertyModel::SetProperties(QVector<PropertySpec> properties) {
  QVector<QPair<QString, QString>> errors;
  beginResetModel();
  root_ = std::make_unique<Node>();
  root_->spec.type = PropertyType::kGroup;
  nodes_by_id_.clear();
  AppendNodes(root_.get(), std::move(properties), &errors);
  endResetModel();
  QPointer<PropertyModel> guard(this);
  for (const auto& error : errors) {
    emit ValidationFailed(error.first, error.second);
    if (guard == nullptr) {
      return;
    }
  }
}

QVariant PropertyModel::Value(const QString& id) const {
  const Node* node = nodes_by_id_.value(id, nullptr);
  return node == nullptr ? QVariant() : node->spec.value;
}

bool PropertyModel::SetValue(const QString& id, const QVariant& value) {
  Node* node = nodes_by_id_.value(id, nullptr);
  return node != nullptr && SetNodeValue(node, value, true);
}

void PropertyModel::SetValues(const QHash<QString, QVariant>& values) {
  QPointer<PropertyModel> guard(this);
  for (auto it = values.constBegin(); it != values.constEnd(); ++it) {
    Node* node = nodes_by_id_.value(it.key(), nullptr);
    if (node != nullptr) {
      SetNodeValue(node, it.value(), true);
      if (guard == nullptr) {
        return;
      }
    }
  }
}

PropertyModel::Node* PropertyModel::NodeForIndex(
    const QModelIndex& index) const {
  return index.isValid() ? static_cast<Node*>(index.internalPointer())
                         : root_.get();
}

QModelIndex PropertyModel::IndexForNode(const Node* node, int column) const {
  if (node == nullptr || node == root_.get()) {
    return {};
  }
  return createIndex(node->Row(), column, const_cast<Node*>(node));
}

void PropertyModel::AppendNodes(Node* parent, QVector<PropertySpec> specs,
                                QVector<QPair<QString, QString>>* errors) {
  parent->children.reserve(static_cast<std::size_t>(specs.size()));
  for (PropertySpec& spec : specs) {
    auto node = std::make_unique<Node>();
    node->parent = parent;
    QVector<PropertySpec> children = std::move(spec.children);
    spec.children.clear();
    node->spec = std::move(spec);

    QVariant normalized;
    QString error;
    if (node->spec.type != PropertyType::kGroup &&
        NormalizeValue(*node, node->spec.value, &normalized, &error)) {
      node->spec.value = normalized;
    } else if (node->spec.type != PropertyType::kGroup) {
      errors->push_back({node->spec.id, error});
      const QVariant fallback = DefaultPropertyValue(node->spec);
      if (NormalizeValue(*node, fallback, &normalized, &error)) {
        node->spec.value = normalized;
      }
    }
    Node* raw_node = node.get();
    parent->children.push_back(std::move(node));
    if (!raw_node->spec.id.isEmpty()) {
      if (nodes_by_id_.contains(raw_node->spec.id)) {
        errors->push_back(
            {raw_node->spec.id,
             tr("Duplicate property id: %1").arg(raw_node->spec.id)});
        raw_node->spec.id.clear();
      } else {
        nodes_by_id_.insert(raw_node->spec.id, raw_node);
      }
    }
    AppendNodes(raw_node, std::move(children), errors);
  }
}

bool PropertyModel::NormalizeValue(const Node& node, const QVariant& input,
                                   QVariant* output, QString* error) const {
  bool ok = true;
  QVariant normalized;
  switch (node.spec.type) {
    case PropertyType::kString:
      normalized = input.toString();
      break;
    case PropertyType::kBoolean:
      normalized = input.toBool();
      break;
    case PropertyType::kInteger: {
      const qlonglong value = input.toLongLong(&ok);
      normalized = value;
      break;
    }
    case PropertyType::kDouble: {
      const double value = input.toDouble(&ok);
      ok = ok && std::isfinite(value);
      normalized = value;
      break;
    }
    case PropertyType::kColor: {
      QColor color;
      if (input.canConvert<QColor>()) {
        color = input.value<QColor>();
      }
      if (!color.isValid()) {
        color = QColor(input.toString());
      }
      ok = color.isValid();
      normalized = color;
      break;
    }
    case PropertyType::kGroup:
      ok = false;
      break;
  }
  if (!ok) {
    *error = tr("Invalid value for %1").arg(node.spec.name);
    return false;
  }

  if (node.spec.type == PropertyType::kInteger) {
    const qlonglong numeric_value = normalized.toLongLong();
    if (node.spec.minimum.isValid() &&
        numeric_value < node.spec.minimum.toLongLong()) {
      *error = tr("%1 is below the minimum").arg(node.spec.name);
      return false;
    }
    if (node.spec.maximum.isValid() &&
        numeric_value > node.spec.maximum.toLongLong()) {
      *error = tr("%1 is above the maximum").arg(node.spec.name);
      return false;
    }
  } else if (node.spec.type == PropertyType::kDouble) {
    const double numeric_value = normalized.toDouble();
    if (node.spec.minimum.isValid() &&
        numeric_value < node.spec.minimum.toDouble()) {
      *error = tr("%1 is below the minimum").arg(node.spec.name);
      return false;
    }
    if (node.spec.maximum.isValid() &&
        numeric_value > node.spec.maximum.toDouble()) {
      *error = tr("%1 is above the maximum").arg(node.spec.name);
      return false;
    }
  }
  *output = normalized;
  return true;
}

bool PropertyModel::SetNodeValue(Node* node, const QVariant& value,
                                 bool emit_value_signal) {
  QVariant normalized;
  QString error;
  if (!NormalizeValue(*node, value, &normalized, &error)) {
    emit ValidationFailed(node->spec.id, error);
    return false;
  }
  if (node->spec.value == normalized) {
    return true;
  }
  node->spec.value = normalized;
  const QString property_id = node->spec.id;
  const QVariant emitted_value = normalized;
  const QModelIndex value_index = IndexForNode(node);
  QPointer<PropertyModel> guard(this);
  emit dataChanged(
      value_index, value_index,
      {Qt::DisplayRole, Qt::EditRole, Qt::CheckStateRole, Qt::DecorationRole});
  if (emit_value_signal && guard != nullptr) {
    emit ValueChanged(property_id, emitted_value);
  }
  return true;
}

PropertyDelegate::PropertyDelegate(QObject* parent)
    : QStyledItemDelegate(parent) {}

QWidget* PropertyDelegate::createEditor(QWidget* parent,
                                        const QStyleOptionViewItem& option,
                                        const QModelIndex& index) const {
  Q_UNUSED(option);
  if (index.column() != 1 ||
      index.data(PropertyModel::kReadOnlyRole).toBool()) {
    return nullptr;
  }
  const auto type =
      static_cast<PropertyType>(index.data(PropertyModel::kTypeRole).toInt());
  switch (type) {
    case PropertyType::kString:
    case PropertyType::kColor:
      return new QLineEdit(parent);
    case PropertyType::kInteger: {
      auto* editor = new QLineEdit(parent);
      editor->setValidator(new QRegularExpressionValidator(
          QRegularExpression(QStringLiteral(R"([+-]?\d+)")), editor));
      return editor;
    }
    case PropertyType::kDouble: {
      auto* editor = new QDoubleSpinBox(parent);
      editor->setDecimals(6);
      editor->setRange(index.data(PropertyModel::kMinimumRole).isValid()
                           ? index.data(PropertyModel::kMinimumRole).toDouble()
                           : -std::numeric_limits<double>::max(),
                       index.data(PropertyModel::kMaximumRole).isValid()
                           ? index.data(PropertyModel::kMaximumRole).toDouble()
                           : std::numeric_limits<double>::max());
      if (index.data(PropertyModel::kStepRole).isValid()) {
        editor->setSingleStep(index.data(PropertyModel::kStepRole).toDouble());
      }
      return editor;
    }
    case PropertyType::kBoolean:
    case PropertyType::kGroup:
      return nullptr;
  }
  return nullptr;
}

void PropertyDelegate::setEditorData(QWidget* editor,
                                     const QModelIndex& index) const {
  const QVariant value = index.data(Qt::EditRole);
  if (auto* line_edit = qobject_cast<QLineEdit*>(editor)) {
    if (static_cast<PropertyType>(
            index.data(PropertyModel::kTypeRole).toInt()) ==
        PropertyType::kColor) {
      line_edit->setText(value.value<QColor>().name(QColor::HexArgb));
    } else {
      line_edit->setText(value.toString());
    }
  } else if (auto* double_editor = qobject_cast<QDoubleSpinBox*>(editor)) {
    double_editor->setValue(value.toDouble());
  }
}

void PropertyDelegate::setModelData(QWidget* editor, QAbstractItemModel* model,
                                    const QModelIndex& index) const {
  QVariant value;
  if (auto* line_edit = qobject_cast<QLineEdit*>(editor)) {
    value = line_edit->text();
  } else if (auto* double_editor = qobject_cast<QDoubleSpinBox*>(editor)) {
    value = double_editor->value();
  } else {
    return;
  }
  model->setData(index, value, Qt::EditRole);
}

void PropertyDelegate::updateEditorGeometry(QWidget* editor,
                                            const QStyleOptionViewItem& option,
                                            const QModelIndex& index) const {
  Q_UNUSED(index);
  editor->setGeometry(option.rect);
}

PropertyEditor::PropertyEditor(QWidget* parent) : QTreeView(parent) {
  setObjectName(QStringLiteral("AutovizPropertyEditor"));
  setProperty("autovizRole", QStringLiteral("propertyEditor"));
  setAccessibleName(tr("Property editor"));
  setAlternatingRowColors(true);
  setFrameShape(QFrame::NoFrame);
  setAllColumnsShowFocus(true);
  setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  setAnimated(false);
  setUniformRowHeights(true);
  setRootIsDecorated(true);
  setItemsExpandable(true);
  setEditTriggers(QAbstractItemView::DoubleClicked |
                  QAbstractItemView::EditKeyPressed |
                  QAbstractItemView::SelectedClicked);

  model_ = new PropertyModel(this);
  delegate_ = new PropertyDelegate(this);
  setModel(model_);
  setItemDelegate(delegate_);
  header()->setHighlightSections(false);
  header()->setStretchLastSection(true);
  header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  header()->setSectionResizeMode(1, QHeaderView::Stretch);
  ApplyWidgetTheme(this, tone_);

  connect(model_, &PropertyModel::ValueChanged, this,
          &PropertyEditor::PropertyValueChanged);
  connect(model_, &PropertyModel::ValidationFailed, this,
          &PropertyEditor::PropertyValidationFailed);
}

void PropertyEditor::SetProperties(QVector<PropertySpec> properties) {
  model_->SetProperties(std::move(properties));
  expandAll();
}

void PropertyEditor::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyWidgetTheme(this, tone_);
}

}  // namespace autoviz::widget
