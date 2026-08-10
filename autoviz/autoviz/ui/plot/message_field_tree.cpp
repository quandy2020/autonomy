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

const google::protobuf::Descriptor* FindGeneratedDescriptor(
    const std::string& type_name) {
  const google::protobuf::DescriptorPool* pool =
      google::protobuf::DescriptorPool::generated_pool();
  if (pool == nullptr || type_name.empty()) {
    return nullptr;
  }
  if (const google::protobuf::Descriptor* desc =
          pool->FindMessageTypeByName(type_name)) {
    return desc;
  }
  return pool->FindMessageTypeByName(StripPackagePrefix(type_name));
}

const google::protobuf::Descriptor* ResolveDescriptor(
    const std::string& message_type) {
  if (message_type.empty()) {
    return nullptr;
  }
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  if (const google::protobuf::Descriptor* desc = FindGeneratedDescriptor(normalized)) {
    return desc;
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

void CollectNumericFieldPaths(const google::protobuf::Descriptor* desc,
                              const QString& path_prefix, QStringList* out) {
  if (desc == nullptr || out == nullptr) {
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
      CollectNumericFieldPaths(field->message_type(), path, out);
    } else if (IsNumericField(field)) {
      out->push_back(path);
    }
  }
}

QString StripArrayIndexSuffix(QString segment) {
  const int bracket = segment.indexOf(QLatin1Char('['));
  if (bracket > 0) {
    segment.truncate(bracket);
  }
  return segment;
}

const google::protobuf::Descriptor* DescriptorAtRelativePath(
    const google::protobuf::Descriptor* root, const QString& relative_path) {
  if (root == nullptr) {
    return nullptr;
  }
  const QString trimmed = relative_path.trimmed();
  if (trimmed.isEmpty()) {
    return root;
  }
  const QStringList segments = trimmed.split(QLatin1Char('.'), Qt::SkipEmptyParts);
  const google::protobuf::Descriptor* desc = root;
  for (const QString& raw_segment : segments) {
    const QString segment = StripArrayIndexSuffix(raw_segment);
    const google::protobuf::FieldDescriptor* field =
        desc->FindFieldByName(segment.toStdString());
    if (field == nullptr ||
        field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      return nullptr;
    }
    // Repeated message fields drill into the element type (Foxglove-style poses.*).
    desc = field->message_type();
  }
  return desc;
}

void CollectAllPlotFieldPaths(const google::protobuf::Descriptor* desc,
                              const QString& path_prefix, QStringList* out) {
  if (desc == nullptr || out == nullptr) {
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
    out->push_back(path);
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      CollectAllPlotFieldPaths(field->message_type(), path, out);
    }
  }
}

void CollectPlotBrowsePaths(const google::protobuf::Descriptor* desc,
                            const QString& path_prefix, QStringList* out) {
  if (desc == nullptr || out == nullptr) {
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
      out->push_back(path);
      CollectPlotBrowsePaths(field->message_type(), path, out);
    } else if (IsNumericField(field)) {
      out->push_back(path);
    }
  }
}

}  // namespace

QStringList PlotBrowsePathsForMessageType(const std::string& message_type) {
  QStringList paths;
  CollectPlotBrowsePaths(ResolveDescriptor(message_type), QString(), &paths);
  paths.sort(Qt::CaseInsensitive);
  return paths;
}

QStringList PlotAllFieldPathsForMessageType(const std::string& message_type) {
  QStringList paths;
  CollectAllPlotFieldPaths(ResolveDescriptor(message_type), QString(), &paths);
  paths.sort(Qt::CaseInsensitive);
  paths.removeDuplicates();
  return paths;
}

QStringList PlotNextLevelFieldPaths(const std::string& message_type,
                                    const QString& parent_field_path) {
  QStringList out;
  const google::protobuf::Descriptor* desc = DescriptorAtRelativePath(
      ResolveDescriptor(message_type), parent_field_path.trimmed());
  if (desc == nullptr) {
    return out;
  }
  for (int i = 0; i < desc->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = desc->field(i);
    if (field == nullptr) {
      continue;
    }
    out.push_back(ProtobufToQString(field->name()));
  }
  out.sort(Qt::CaseInsensitive);
  return out;
}

QStringList NumericFieldPathsForMessageType(const std::string& message_type) {
  QStringList paths;
  CollectNumericFieldPaths(ResolveDescriptor(message_type), QString(), &paths);
  paths.sort(Qt::CaseInsensitive);
  return paths;
}

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
