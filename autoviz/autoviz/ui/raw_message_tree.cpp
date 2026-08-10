/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/raw_message_tree.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <QAction>
#include <QAbstractItemView>
#include <QContextMenuEvent>
#include <QDrag>
#include <QMenu>
#include <QMouseEvent>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <optional>

#include "autoviz/common/protobuf_qt_string.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"
#include "autoviz/ui/plot/message_path_navigation.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"

namespace autoviz {
namespace raw_messages {
namespace {

using automsgs::msgs::DynamicFactory;

constexpr int kPathRole = Qt::UserRole + 100;
constexpr int kMaxRepeatedElements = 64;

void AppendMessageNode(QTreeWidgetItem* parent, const google::protobuf::Message& message,
                       const QString& path_prefix, const QString& channel);
void AppendSchemaMessage(QTreeWidgetItem* parent, const google::protobuf::Descriptor* desc,
                         const QString& path_prefix, const QString& channel);

bool IsNumericField(const google::protobuf::FieldDescriptor* field) {
  if (field == nullptr) {
    return false;
  }
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return true;
    default:
      return false;
  }
}

void ApplyPlotDragMetadata(QTreeWidgetItem* item, const QString& channel,
                           const QString& field_path, bool draggable) {
  if (item == nullptr) {
    return;
  }
  item->setData(0, plot::kTopicChannelRole, channel);
  item->setData(0, plot::kTopicFieldPathRole, field_path);
  item->setData(0, plot::kTopicDraggableRole, draggable);
  if (draggable) {
    item->setFlags(item->flags() | Qt::ItemIsDragEnabled);
    item->setToolTip(
        0, QStringLiteral("Drag or right-click to add this field to Plot"));
  }
}

std::string StripPackagePrefix(const std::string& type_name) {
  static const char* kPrefix = "automsgs.msgs.";
  if (type_name.rfind(kPrefix, 0) == 0) {
    return type_name.substr(std::char_traits<char>::length(kPrefix));
  }
  return type_name;
}

const google::protobuf::Descriptor* ResolveDescriptor(
    const std::string& message_type) {
  if (message_type.empty()) {
    return nullptr;
  }
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  const google::protobuf::DescriptorPool* pool =
      google::protobuf::DescriptorPool::generated_pool();
  if (pool != nullptr) {
    if (const google::protobuf::Descriptor* desc =
            pool->FindMessageTypeByName(normalized)) {
      return desc;
    }
    if (const google::protobuf::Descriptor* desc =
            pool->FindMessageTypeByName(StripPackagePrefix(normalized))) {
      return desc;
    }
  }

  static DynamicFactory factory;
  DynamicFactory::MessagePtr message = factory.New(normalized);
  if (message == nullptr) {
    message = factory.New(StripPackagePrefix(normalized));
  }
  if (message == nullptr) {
    return nullptr;
  }
  return message->GetDescriptor();
}

QString FieldTypeLabel(const google::protobuf::FieldDescriptor* field) {
  if (field == nullptr) {
    return QString();
  }
  if (field->is_repeated()) {
    return QStringLiteral("%1[]").arg(QString::fromUtf8(field->type_name()));
  }
  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return ProtobufToQString(field->message_type()->name());
  }
  return QString::fromUtf8(field->type_name());
}

QString FormatScalarValue(const google::protobuf::Message& message,
                          const google::protobuf::FieldDescriptor* field,
                          int repeated_index) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  const bool repeated = field->is_repeated();
  const int index = repeated ? repeated_index : 0;

  auto read_string = [&]() -> QString {
    const std::string value =
        repeated ? reflection->GetRepeatedString(message, field, index)
                 : reflection->GetString(message, field);
    if (value.size() > 120) {
      return QStringLiteral("\"%1…\" (%2 bytes)")
          .arg(QString::fromStdString(value.substr(0, 117)),
               QString::number(static_cast<qulonglong>(value.size())));
    }
    return QStringLiteral("\"%1\"").arg(QString::fromStdString(value));
  };

  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return QString::number(repeated ? reflection->GetRepeatedDouble(message, field, index)
                                      : reflection->GetDouble(message, field),
                             'g', 8);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return QString::number(repeated ? reflection->GetRepeatedFloat(message, field, index)
                                      : reflection->GetFloat(message, field),
                             'g', 6);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return QString::number(repeated ? reflection->GetRepeatedInt32(message, field, index)
                                      : reflection->GetInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return QString::number(repeated ? reflection->GetRepeatedInt64(message, field, index)
                                      : reflection->GetInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return QString::number(repeated ? reflection->GetRepeatedUInt32(message, field, index)
                                      : reflection->GetUInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return QString::number(repeated ? reflection->GetRepeatedUInt64(message, field, index)
                                      : reflection->GetUInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return (repeated ? reflection->GetRepeatedBool(message, field, index)
                       : reflection->GetBool(message, field))
                 ? QStringLiteral("true")
                 : QStringLiteral("false");
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return read_string();
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const google::protobuf::EnumValueDescriptor* enum_value =
          repeated ? reflection->GetRepeatedEnum(message, field, index)
                   : reflection->GetEnum(message, field);
      return enum_value != nullptr ? ProtobufToQString(enum_value->name())
                                   : QStringLiteral("(unknown enum)");
    }
    default:
      return QStringLiteral("—");
  }
}

bool FieldHasValue(const google::protobuf::Message& message,
                   const google::protobuf::FieldDescriptor* field) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  if (field->is_repeated()) {
    return reflection->FieldSize(message, field) > 0;
  }
  return reflection->HasField(message, field);
}

void AppendFieldNode(QTreeWidgetItem* parent, const google::protobuf::Message& message,
                     const google::protobuf::FieldDescriptor* field,
                     const QString& path_prefix, const QString& channel) {
  if (parent == nullptr || field == nullptr) {
    return;
  }

  const QString segment = ProtobufToQString(field->name());
  const QString path =
      path_prefix.isEmpty() ? segment : path_prefix + QLatin1Char('.') + segment;
  const google::protobuf::Reflection* reflection = message.GetReflection();

  if (field->is_repeated()) {
    const int count = reflection->FieldSize(message, field);
    auto* array_item =
        new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field),
                                     count > 0 ? QString::number(count) : QStringLiteral("[]")});
    array_item->setData(0, kPathRole, path);
    if (count == 0) {
      return;
    }

    const int limit = std::min(count, kMaxRepeatedElements);
    for (int i = 0; i < limit; ++i) {
      const QString index_label = QStringLiteral("[%1]").arg(i);
      const QString index_path = path + index_label;
      if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        const google::protobuf::Message& element =
            reflection->GetRepeatedMessage(message, field, i);
        auto* element_item = new QTreeWidgetItem(
            array_item,
            {index_label, ProtobufToQString(element.GetDescriptor()->name()), QString()});
        element_item->setData(0, kPathRole, index_path);
        AppendMessageNode(element_item, element, index_path, channel);
      } else {
        auto* element_item = new QTreeWidgetItem(
            array_item, {index_label, FieldTypeLabel(field),
                         FormatScalarValue(message, field, i)});
        element_item->setData(0, kPathRole, index_path);
        ApplyPlotDragMetadata(element_item, channel, index_path,
                              IsNumericField(field));
      }
    }
    if (count > limit) {
      auto* more_item = new QTreeWidgetItem(
          array_item,
          {QStringLiteral("…"), QString(),
           QStringLiteral("+%1 more").arg(count - limit)});
      more_item->setFlags(more_item->flags() & ~Qt::ItemIsSelectable);
    }
    return;
  }

  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    if (!FieldHasValue(message, field)) {
      auto* item = new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field), QString()});
      item->setData(0, kPathRole, path);
      return;
    }
    const google::protobuf::Message& child =
        reflection->GetMessage(message, field);
    auto* item = new QTreeWidgetItem(
        parent, {segment, ProtobufToQString(child.GetDescriptor()->name()), QString()});
    item->setData(0, kPathRole, path);
    AppendMessageNode(item, child, path, channel);
    return;
  }

  const QString value =
      FieldHasValue(message, field) ? FormatScalarValue(message, field, -1)
                                    : QStringLiteral("—");
  auto* item = new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field), value});
  item->setData(0, kPathRole, path);
  ApplyPlotDragMetadata(item, channel, path, IsNumericField(field));
}

void AppendMessageNode(QTreeWidgetItem* parent, const google::protobuf::Message& message,
                       const QString& path_prefix, const QString& channel) {
  const google::protobuf::Descriptor* desc = message.GetDescriptor();
  if (parent == nullptr || desc == nullptr) {
    return;
  }
  for (int i = 0; i < desc->field_count(); ++i) {
    AppendFieldNode(parent, message, desc->field(i), path_prefix, channel);
  }
}

const google::protobuf::Message* NavigateMessagePath(
    const google::protobuf::Message& root, const std::string& path) {
  if (path.empty()) {
    return &root;
  }

  const std::vector<plot::MessagePathSegment> segments = plot::ParseMessagePath(path);
  if (segments.empty()) {
    return nullptr;
  }

  const google::protobuf::Message* current = &root;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    const plot::MessagePathSegment& segment = segments[i];
    const google::protobuf::FieldDescriptor* field =
        current->GetDescriptor()->FindFieldByName(segment.field);
    if (field == nullptr) {
      return nullptr;
    }

    const google::protobuf::Reflection* reflection = current->GetReflection();
    if (field->is_repeated()) {
      if (!segment.has_bracket || segment.all_elements || segment.filter.has_value()) {
        return nullptr;
      }
      if (reflection->FieldSize(*current, field) <= segment.index) {
        return nullptr;
      }
      if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        current = &reflection->GetRepeatedMessage(*current, field, segment.index);
        continue;
      }
      return i + 1 == segments.size() ? nullptr : nullptr;
    }

    if (i + 1 == segments.size()) {
      if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE &&
          FieldHasValue(*current, field)) {
        return &reflection->GetMessage(*current, field);
      }
      return nullptr;
    }

    if (field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE ||
        !FieldHasValue(*current, field)) {
      return nullptr;
    }
    current = &reflection->GetMessage(*current, field);
  }
  return current;
}

void AppendSchemaField(QTreeWidgetItem* parent,
                       const google::protobuf::FieldDescriptor* field,
                       const QString& path_prefix, const QString& channel) {
  if (parent == nullptr || field == nullptr) {
    return;
  }
  const QString segment = ProtobufToQString(field->name());
  const QString path =
      path_prefix.isEmpty() ? segment : path_prefix + QLatin1Char('.') + segment;

  if (field->is_repeated()) {
    auto* item = new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field), QStringLiteral("[]")});
    item->setData(0, kPathRole, path);
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      auto* element = new QTreeWidgetItem(
          item, {QStringLiteral("[0]"), ProtobufToQString(field->message_type()->name()),
                QStringLiteral("—")});
      element->setData(0, kPathRole, path + QStringLiteral("[0]"));
      AppendSchemaMessage(element, field->message_type(), path + QStringLiteral("[0]"),
                          channel);
    } else {
      ApplyPlotDragMetadata(item, channel, path, IsNumericField(field));
    }
    return;
  }

  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    auto* item = new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field), QString()});
    item->setData(0, kPathRole, path);
    AppendSchemaMessage(item, field->message_type(), path, channel);
    return;
  }

  auto* item = new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field), QStringLiteral("—")});
  item->setData(0, kPathRole, path);
  ApplyPlotDragMetadata(item, channel, path, IsNumericField(field));
}

void AppendSchemaMessage(QTreeWidgetItem* parent,
                         const google::protobuf::Descriptor* desc,
                         const QString& path_prefix, const QString& channel) {
  if (parent == nullptr || desc == nullptr) {
    return;
  }
  for (int i = 0; i < desc->field_count(); ++i) {
    AppendSchemaField(parent, desc->field(i), path_prefix, channel);
  }
}

void ExpandFirstLevel(QTreeWidget* tree) {
  if (tree == nullptr) {
    return;
  }
  tree->expandToDepth(1);
}

QString ItemPath(QTreeWidgetItem* item) {
  return item != nullptr ? item->data(0, kPathRole).toString() : QString();
}

QString DisplayValueAtPath(const google::protobuf::Message& message, const QString& path) {
  if (path.isEmpty()) {
    return QString();
  }
  const std::optional<std::string> value =
      plot::FormatMessagePathValue(message, path.toStdString());
  if (!value.has_value()) {
    return QString();
  }
  return QString::fromStdString(*value);
}

void UpdateItemValuesRecursive(QTreeWidgetItem* item,
                               const google::protobuf::Message& message) {
  if (item == nullptr) {
    return;
  }
  if (item->childCount() == 0) {
    const QString path = ItemPath(item);
    const QString value = DisplayValueAtPath(message, path);
    if (!value.isEmpty()) {
      item->setText(2, value);
    }
    return;
  }

  const QString path = ItemPath(item);
  const QString own_value = DisplayValueAtPath(message, path);
  if (!own_value.isEmpty()) {
    item->setText(2, own_value);
  }

  for (int i = 0; i < item->childCount(); ++i) {
    UpdateItemValuesRecursive(item->child(i), message);
  }
}

void CollectExpandedPathsRecursive(QTreeWidgetItem* item, QSet<QString>* paths) {
  if (item == nullptr || paths == nullptr) {
    return;
  }
  if (item->isExpanded()) {
    const QString path = ItemPath(item);
    if (!path.isEmpty()) {
      paths->insert(path);
    }
  }
  for (int i = 0; i < item->childCount(); ++i) {
    CollectExpandedPathsRecursive(item->child(i), paths);
  }
}

}  // namespace

RawMessageTreeWidget::RawMessageTreeWidget(QWidget* parent) : QTreeWidget(parent) {
  setDragEnabled(true);
  setDragDropMode(QAbstractItemView::DragOnly);
  setDefaultDropAction(Qt::CopyAction);
}

void RawMessageTreeWidget::setActiveChannel(const QString& channel) {
  active_channel_ = channel;
}

std::optional<plot::PlotSeriesDragPayload> RawMessageTreeWidget::payloadFromItem(
    QTreeWidgetItem* item) const {
  if (item == nullptr || active_channel_.isEmpty()) {
    return std::nullopt;
  }
  if (!item->data(0, plot::kTopicDraggableRole).toBool()) {
    return std::nullopt;
  }
  plot::PlotSeriesDragPayload payload;
  payload.channel = item->data(0, plot::kTopicChannelRole).toString();
  if (payload.channel.isEmpty()) {
    payload.channel = active_channel_;
  }
  payload.field_path = item->data(0, plot::kTopicFieldPathRole).toString();
  if (payload.field_path.isEmpty()) {
    return std::nullopt;
  }
  return payload;
}

void RawMessageTreeWidget::requestAddToPlot(QTreeWidgetItem* item) {
  const std::optional<plot::PlotSeriesDragPayload> payload = payloadFromItem(item);
  if (!payload.has_value()) {
    return;
  }
  emit addToPlotRequested(payload->channel, payload->field_path);
}

void RawMessageTreeWidget::startDrag(Qt::DropActions supported_actions) {
  QVector<plot::PlotSeriesDragPayload> payloads;
  const QList<QTreeWidgetItem*> selected = selectedItems();
  if (!selected.isEmpty()) {
    for (QTreeWidgetItem* item : selected) {
      if (const std::optional<plot::PlotSeriesDragPayload> payload = payloadFromItem(item)) {
        payloads.push_back(*payload);
      }
    }
  } else if (QTreeWidgetItem* item = currentItem()) {
    if (const std::optional<plot::PlotSeriesDragPayload> payload = payloadFromItem(item)) {
      payloads.push_back(*payload);
    }
  }
  if (payloads.isEmpty()) {
    return;
  }
  auto* drag = new QDrag(this);
  drag->setMimeData(plot::MakePlotSeriesListDragPayload(payloads));
  drag->exec(supported_actions, Qt::CopyAction);
}

void RawMessageTreeWidget::contextMenuEvent(QContextMenuEvent* event) {
  QTreeWidgetItem* item = itemAt(event->pos());
  if (item == nullptr || !payloadFromItem(item).has_value()) {
    QTreeWidget::contextMenuEvent(event);
    return;
  }
  QMenu menu(this);
  QAction* add_action = menu.addAction(tr("Add to Plot"));
  QAction* chosen = menu.exec(event->globalPos());
  if (chosen == add_action) {
    requestAddToPlot(item);
  }
}

void RawMessageTreeWidget::toggleSubtree(QTreeWidgetItem* item) {
  if (item == nullptr) {
    return;
  }
  if (SubtreeFullyExpanded(item)) {
    item->setExpanded(false);
    CollapseSubtree(item);
    return;
  }
  ExpandSubtree(item);
}

void RawMessageTreeWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  QTreeWidgetItem* item = itemAt(event->pos());
  if (item != nullptr && item->childCount() > 0) {
    toggleSubtree(item);
    event->accept();
    return;
  }
  QTreeWidget::mouseDoubleClickEvent(event);
}

void ExpandSubtree(QTreeWidgetItem* item) {
  if (item == nullptr) {
    return;
  }
  item->setExpanded(true);
  for (int i = 0; i < item->childCount(); ++i) {
    ExpandSubtree(item->child(i));
  }
}

void CollapseSubtree(QTreeWidgetItem* item) {
  if (item == nullptr) {
    return;
  }
  for (int i = 0; i < item->childCount(); ++i) {
    CollapseSubtree(item->child(i));
    item->child(i)->setExpanded(false);
  }
}

bool SubtreeFullyExpanded(QTreeWidgetItem* item) {
  if (item == nullptr) {
    return true;
  }
  if (item->childCount() > 0 && !item->isExpanded()) {
    return false;
  }
  for (int i = 0; i < item->childCount(); ++i) {
    if (!SubtreeFullyExpanded(item->child(i))) {
      return false;
    }
  }
  return true;
}

void ExpandAllNodes(QTreeWidget* tree) {
  if (tree == nullptr) {
    return;
  }
  for (int i = 0; i < tree->topLevelItemCount(); ++i) {
    ExpandSubtree(tree->topLevelItem(i));
  }
}

void CollapseAllNodes(QTreeWidget* tree) {
  if (tree == nullptr) {
    return;
  }
  for (int i = 0; i < tree->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = tree->topLevelItem(i);
    if (item == nullptr) {
      continue;
    }
    item->setExpanded(false);
    CollapseSubtree(item);
  }
}

void PopulateMessageTree(QTreeWidget* tree, const google::protobuf::Message& message,
                         const QString& root_label, const QString& channel,
                         const QString& path_filter, bool apply_initial_expand) {
  if (tree == nullptr) {
    return;
  }

  const QString trimmed_path = path_filter.trimmed();
  if (trimmed_path.isEmpty() &&
      UpdateMessageTreeValues(tree, message, root_label, trimmed_path)) {
    return;
  }

  const QSet<QString> expanded = CaptureExpandedPaths(tree);
  tree->clear();

  if (!trimmed_path.isEmpty()) {
    if (const google::protobuf::Message* sub =
            NavigateMessagePath(message, trimmed_path.toStdString())) {
      auto* root = new QTreeWidgetItem(tree, {trimmed_path, root_label, QString()});
      root->setData(0, kPathRole, trimmed_path);
      AppendMessageNode(root, *sub, trimmed_path, channel);
      RestoreExpandedPaths(tree, expanded);
      return;
    }

    const std::optional<std::string> value =
        plot::FormatMessagePathValue(message, trimmed_path.toStdString());
    auto* root = new QTreeWidgetItem(
        tree, {trimmed_path, QStringLiteral("value"),
               value.has_value() ? QString::fromStdString(*value)
                                 : QStringLiteral("(no value)")});
    root->setData(0, kPathRole, trimmed_path);
    RestoreExpandedPaths(tree, expanded);
    return;
  }

  auto* root = new QTreeWidgetItem(tree, {root_label, QString(), QString()});
  root->setData(0, kPathRole, root_label);
  AppendMessageNode(root, message, QString(), channel);
  RestoreExpandedPaths(tree, expanded);
  if (apply_initial_expand && expanded.isEmpty()) {
    ExpandFirstLevel(tree);
  }
}

bool UpdateMessageTreeValues(QTreeWidget* tree, const google::protobuf::Message& message,
                             const QString& root_label, const QString& path_filter) {
  if (tree == nullptr || !path_filter.trimmed().isEmpty() ||
      tree->topLevelItemCount() != 1) {
    return false;
  }

  QTreeWidgetItem* root = tree->topLevelItem(0);
  if (root == nullptr || ItemPath(root) != root_label) {
    return false;
  }

  UpdateItemValuesRecursive(root, message);
  return true;
}

void PopulateSchemaTree(QTreeWidget* tree, const std::string& message_type,
                        const QString& channel) {
  if (tree == nullptr) {
    return;
  }
  tree->clear();
  const google::protobuf::Descriptor* desc = ResolveDescriptor(message_type);
  if (desc == nullptr) {
    auto* item = new QTreeWidgetItem(
        tree, {QStringLiteral("(unknown schema)"), QString(), QStringLiteral("—")});
    item->setData(0, kPathRole, QStringLiteral("schema"));
    return;
  }

  const QString root_label = ProtobufToQString(desc->name());
  auto* root = new QTreeWidgetItem(tree, {root_label, QString(), QString()});
  root->setData(0, kPathRole, root_label);
  AppendSchemaMessage(root, desc, QString(), channel);
  ExpandFirstLevel(tree);
}

QSet<QString> CaptureExpandedPaths(QTreeWidget* tree) {
  QSet<QString> expanded;
  if (tree == nullptr) {
    return expanded;
  }
  for (int i = 0; i < tree->topLevelItemCount(); ++i) {
    CollectExpandedPathsRecursive(tree->topLevelItem(i), &expanded);
  }
  return expanded;
}

void RestoreExpandedPaths(QTreeWidget* tree, const QSet<QString>& expanded_paths) {
  if (tree == nullptr || expanded_paths.isEmpty()) {
    return;
  }

  std::function<QTreeWidgetItem*(QTreeWidgetItem*, const QString&)> find_path;
  find_path = [&](QTreeWidgetItem* item, const QString& target) -> QTreeWidgetItem* {
    if (item == nullptr) {
      return nullptr;
    }
    if (ItemPath(item) == target) {
      return item;
    }
    for (int i = 0; i < item->childCount(); ++i) {
      if (QTreeWidgetItem* found = find_path(item->child(i), target)) {
        return found;
      }
    }
    return nullptr;
  };

  auto expand_ancestors = [](QTreeWidgetItem* item) {
    for (QTreeWidgetItem* parent = item != nullptr ? item->parent() : nullptr; parent != nullptr;
         parent = parent->parent()) {
      parent->setExpanded(true);
    }
    if (item != nullptr) {
      item->setExpanded(true);
    }
  };

  for (int i = 0; i < tree->topLevelItemCount(); ++i) {
    QTreeWidgetItem* root = tree->topLevelItem(i);
    for (const QString& path : expanded_paths) {
      if (QTreeWidgetItem* item = find_path(root, path)) {
        expand_ancestors(item);
      }
    }
  }
}

}  // namespace raw_messages
}  // namespace autoviz
