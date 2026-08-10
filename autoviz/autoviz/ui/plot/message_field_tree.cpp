/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/message_field_tree.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>

#include <QTreeWidgetItem>

#include "autoviz/common/protobuf_qt_string.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {
namespace plot {
namespace {

using automsgs::msgs::DynamicFactory;

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
  static DynamicFactory factory;
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  DynamicFactory::MessagePtr message = factory.New(normalized);
  if (message == nullptr) {
    message = factory.New(StripPackagePrefix(normalized));
  }
  if (message == nullptr) {
    return nullptr;
  }
  return message->GetDescriptor();
}

bool IsNumericField(const google::protobuf::FieldDescriptor* field) {
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

void AppendDescriptorFields(QTreeWidgetItem* parent,
                            const google::protobuf::Descriptor* desc,
                            const QString& path_prefix) {
  if (parent == nullptr || desc == nullptr) {
    return;
  }
  for (int i = 0; i < desc->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = desc->field(i);
    if (field == nullptr || field->is_repeated()) {
      continue;
    }
    const QString segment = ProtobufToQString(field->name());
    const QString path =
        path_prefix.isEmpty() ? segment : path_prefix + QLatin1Char('.') + segment;
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      auto* item = new QTreeWidgetItem(parent, {segment});
      item->setData(0, kTopicFieldPathRole, path);
      item->setData(0, kTopicDraggableRole, false);
      item->setData(0, kTopicTableDraggableRole, false);
      AppendDescriptorFields(item, field->message_type(), path);
      if (item->childCount() == 0) {
        delete item;
      }
    } else if (IsNumericField(field)) {
      auto* item = new QTreeWidgetItem(parent, {segment});
      item->setData(0, kTopicFieldPathRole, path);
      item->setData(0, kTopicDraggableRole, true);
      item->setData(0, kTopicTableDraggableRole, false);
      item->setFlags(item->flags() | Qt::ItemIsDragEnabled);
      item->setToolTip(0, path);
    }
  }
}

void AppendTableArrayFields(QTreeWidgetItem* parent,
                            const google::protobuf::Descriptor* desc,
                            const QString& path_prefix) {
  if (parent == nullptr || desc == nullptr) {
    return;
  }
  for (int i = 0; i < desc->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = desc->field(i);
    if (field == nullptr) {
      continue;
    }
    const QString segment = ProtobufToQString(field->name());
    const QString path =
        path_prefix.isEmpty() ? segment : path_prefix + QLatin1Char('.') + segment;
    if (field->is_repeated()) {
      auto* item = new QTreeWidgetItem(parent, {segment + QStringLiteral(" []")});
      item->setData(0, kTopicFieldPathRole, path);
      item->setData(0, kTopicDraggableRole, false);
      item->setData(0, kTopicTableDraggableRole, true);
      item->setFlags(item->flags() | Qt::ItemIsDragEnabled);
      item->setToolTip(0, QStringLiteral("Table array: %1").arg(path));
      continue;
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      auto* item = new QTreeWidgetItem(parent, {segment});
      item->setData(0, kTopicFieldPathRole, path);
      item->setData(0, kTopicDraggableRole, false);
      item->setData(0, kTopicTableDraggableRole, false);
      AppendTableArrayFields(item, field->message_type(), path);
      if (item->childCount() == 0) {
        delete item;
      }
    }
  }
}

}  // namespace

void PopulateMessageFieldTree(QTreeWidgetItem* parent,
                              const std::string& message_type,
                              const QString& path_prefix) {
  AppendDescriptorFields(parent, ResolveDescriptor(message_type), path_prefix);
}

void PopulateTableArrayFieldTree(QTreeWidgetItem* parent,
                                 const std::string& message_type,
                                 const QString& path_prefix) {
  AppendTableArrayFields(parent, ResolveDescriptor(message_type), path_prefix);
}

}  // namespace plot
}  // namespace autoviz
