/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_field_tree.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/json_util.h>

#include <QComboBox>
#include <QHeaderView>
#include <QMenu>
#include <QStyledItemDelegate>

#include "autoviz/common/protobuf_json_compat.hpp"
#include "autoviz/common/protobuf_qt_string.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/ui/plot/message_path_navigation.hpp"
#include "autoviz/ui/publish/publish_message_codec.hpp"

namespace autoviz {
namespace publish_panel {
namespace {

using automsgs::msgs::DynamicFactory;

constexpr int kPathRole = Qt::UserRole + 200;
constexpr int kEditableRole = Qt::UserRole + 201;
constexpr int kArrayRole = Qt::UserRole + 202;
constexpr int kArrayIndexRole = Qt::UserRole + 203;
constexpr int kBoolRole = Qt::UserRole + 204;
constexpr int kEnumRole = Qt::UserRole + 205;
constexpr int kEnumOptionsRole = Qt::UserRole + 206;
constexpr int kElementRole = Qt::UserRole + 207;
constexpr int kEntryIdRole = Qt::UserRole + 208;
constexpr int kPublisherRootRole = Qt::UserRole + 209;

class PublishFieldValueDelegate : public QStyledItemDelegate {
 public:
  explicit PublishFieldValueDelegate(PublishFieldTreeWidget* tree,
                                     QObject* parent = nullptr)
      : QStyledItemDelegate(parent), tree_(tree) {}

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override {
    QTreeWidgetItem* item =
        tree_ == nullptr ? nullptr : tree_->itemFromIndex(index);
    const int value_col = tree_ == nullptr ? 2 : tree_->editorValueColumn();
    if (item == nullptr || index.column() != value_col ||
        !item->data(0, kEnumRole).toBool()) {
      return QStyledItemDelegate::createEditor(parent, option, index);
    }
    auto* combo = new QComboBox(parent);
    combo->addItems(item->data(0, kEnumOptionsRole).toStringList());
    combo->setFrame(false);
    return combo;
  }

  void setEditorData(QWidget* editor, const QModelIndex& index) const override {
    auto* combo = qobject_cast<QComboBox*>(editor);
    if (combo == nullptr) {
      QStyledItemDelegate::setEditorData(editor, index);
      return;
    }
    const QString value = index.model()->data(index, Qt::EditRole).toString();
    const int found = combo->findText(value);
    combo->setCurrentIndex(found >= 0 ? found : 0);
  }

  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override {
    auto* combo = qobject_cast<QComboBox*>(editor);
    if (combo == nullptr) {
      QStyledItemDelegate::setModelData(editor, model, index);
      return;
    }
    model->setData(index, combo->currentText(), Qt::EditRole);
  }

 private:
  PublishFieldTreeWidget* tree_ = nullptr;
};

QStringList EnumOptionNames(const google::protobuf::EnumDescriptor* enum_desc) {
  QStringList names;
  if (enum_desc == nullptr) {
    return names;
  }
  for (int i = 0; i < enum_desc->value_count(); ++i) {
    names.push_back(ProtobufToQString(enum_desc->value(i)->name()));
  }
  return names;
}

QString ShortTypeLabel(const std::string& type) {
  QString label = QString::fromStdString(type);
  static const QString kPrefix = QStringLiteral("automsgs.msgs.");
  if (label.startsWith(kPrefix)) {
    label = label.mid(kPrefix.size());
  }
  return label;
}

QString MessageToJsonString(const google::protobuf::Message& message) {
  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  SetAlwaysPrintPrimitiveFields(&options);
  options.preserve_proto_field_names = true;
  std::string json;
  if (!google::protobuf::util::MessageToJsonString(message, &json, options).ok()) {
    return {};
  }
  return QString::fromStdString(json);
}

bool LoadJsonIntoMessage(google::protobuf::Message* message,
                         const std::string& message_type, const QString& json) {
  if (message == nullptr) {
    return false;
  }
  message->Clear();
  const QString trimmed = json.trimmed();
  if (trimmed.isEmpty()) {
    return true;
  }
  google::protobuf::util::JsonParseOptions options;
  options.ignore_unknown_fields = true;
  const auto parse_status = google::protobuf::util::JsonStringToMessage(
      trimmed.toStdString(), message, options);
  if (parse_status.ok()) {
    return true;
  }
  const CodecResult encoded =
      PublishMessageCodec::instance().encodeJson(message_type, trimmed);
  if (encoded.ok) {
    return message->ParseFromString(encoded.payload);
  }
  return false;
}

void ConfigureScalarItem(QTreeWidgetItem* item,
                         const google::protobuf::FieldDescriptor* field,
                         const QString& value, int expr_col) {
  item->setText(expr_col, value);
  if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_BOOL) {
    item->setData(0, kBoolRole, true);
    item->setCheckState(expr_col, value == QStringLiteral("true") ? Qt::Checked
                                                                  : Qt::Unchecked);
    item->setFlags((item->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);
    return;
  }
  if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_ENUM) {
    item->setData(0, kEnumRole, true);
    item->setData(0, kEnumOptionsRole, EnumOptionNames(field->enum_type()));
  }
  item->setData(0, kEditableRole, true);
  item->setFlags(item->flags() | Qt::ItemIsEditable);
}

std::string StripPackagePrefix(const std::string& type_name) {
  static const char* kPrefix = "automsgs.msgs.";
  if (type_name.rfind(kPrefix, 0) == 0) {
    return type_name.substr(std::char_traits<char>::length(kPrefix));
  }
  return type_name;
}

std::unique_ptr<google::protobuf::Message> CreateMessage(
    const std::string& message_type) {
  return CreatePublishMessage(message_type);
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

bool FieldHasValue(const google::protobuf::Message& message,
                   const google::protobuf::FieldDescriptor* field) {
  return message.GetReflection()->HasField(message, field);
}

QString FormatScalarValue(const google::protobuf::Message& message,
                          const google::protobuf::FieldDescriptor* field) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return QString::number(reflection->GetDouble(message, field), 'g', 8);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return QString::number(reflection->GetFloat(message, field), 'g', 6);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return QString::number(reflection->GetInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return QString::number(reflection->GetInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return QString::number(reflection->GetUInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return QString::number(reflection->GetUInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetBool(message, field) ? QStringLiteral("true")
                                                 : QStringLiteral("false");
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return QString::fromStdString(reflection->GetString(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      return ProtobufToQString(reflection->GetEnum(message, field)->name());
    default:
      return QString();
  }
}

bool SetScalarValue(google::protobuf::Message* message,
                    const google::protobuf::FieldDescriptor* field,
                    const QString& text) {
  if (message == nullptr || field == nullptr || field->is_repeated() ||
      field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  const google::protobuf::Reflection* reflection = message->GetReflection();
  const QString trimmed = text.trimmed();

  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      reflection->SetDouble(message, field, trimmed.toDouble());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      reflection->SetFloat(message, field, static_cast<float>(trimmed.toDouble()));
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      reflection->SetInt32(message, field, trimmed.toInt());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      reflection->SetInt64(message, field, trimmed.toLongLong());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      reflection->SetUInt32(message, field, trimmed.toUInt());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      reflection->SetUInt64(message, field, trimmed.toULongLong());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: {
      const QString lower = trimmed.toLower();
      reflection->SetBool(message, field,
                          lower == QStringLiteral("true") || lower == QStringLiteral("1") ||
                              lower == QStringLiteral("yes"));
      return true;
    }
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      reflection->SetString(message, field, trimmed.toStdString());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const google::protobuf::EnumDescriptor* enum_desc = field->enum_type();
      if (enum_desc == nullptr) {
        return false;
      }
      const google::protobuf::EnumValueDescriptor* value =
          enum_desc->FindValueByName(trimmed.toStdString());
      if (value == nullptr) {
        value = enum_desc->FindValueByNumber(trimmed.toInt());
      }
      if (value == nullptr) {
        return false;
      }
      reflection->SetEnum(message, field, value);
      return true;
    }
    default:
      return false;
  }
}

QString FormatRepeatedScalarValue(const google::protobuf::Message& message,
                                  const google::protobuf::FieldDescriptor* field,
                                  int index) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  if (index < 0 || index >= reflection->FieldSize(message, field)) {
    return QString();
  }
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return QString::number(reflection->GetRepeatedDouble(message, field, index),
                             'g', 8);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return QString::number(reflection->GetRepeatedFloat(message, field, index),
                             'g', 6);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return QString::number(reflection->GetRepeatedInt32(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return QString::number(reflection->GetRepeatedInt64(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return QString::number(reflection->GetRepeatedUInt32(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return QString::number(reflection->GetRepeatedUInt64(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetRepeatedBool(message, field, index)
                 ? QStringLiteral("true")
                 : QStringLiteral("false");
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return QString::fromStdString(
          reflection->GetRepeatedString(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      return ProtobufToQString(
          reflection->GetRepeatedEnum(message, field, index)->name());
    default:
      return QString();
  }
}

bool SetRepeatedScalarValue(google::protobuf::Message* message,
                            const google::protobuf::FieldDescriptor* field,
                            int index, const QString& text) {
  if (message == nullptr || field == nullptr || !field->is_repeated() ||
      field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  const google::protobuf::Reflection* reflection = message->GetReflection();
  const int count = reflection->FieldSize(*message, field);
  if (index < 0 || index >= count) {
    return false;
  }
  const QString trimmed = text.trimmed();

  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      reflection->SetRepeatedDouble(message, field, index, trimmed.toDouble());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      reflection->SetRepeatedFloat(message, field, index,
                                   static_cast<float>(trimmed.toDouble()));
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      reflection->SetRepeatedInt32(message, field, index, trimmed.toInt());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      reflection->SetRepeatedInt64(message, field, index, trimmed.toLongLong());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      reflection->SetRepeatedUInt32(message, field, index, trimmed.toUInt());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      reflection->SetRepeatedUInt64(message, field, index, trimmed.toULongLong());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: {
      const QString lower = trimmed.toLower();
      reflection->SetRepeatedBool(
          message, field, index,
          lower == QStringLiteral("true") || lower == QStringLiteral("1") ||
              lower == QStringLiteral("yes"));
      return true;
    }
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      reflection->SetRepeatedString(message, field, index, trimmed.toStdString());
      return true;
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const google::protobuf::EnumDescriptor* enum_desc = field->enum_type();
      if (enum_desc == nullptr) {
        return false;
      }
      const google::protobuf::EnumValueDescriptor* value =
          enum_desc->FindValueByName(trimmed.toStdString());
      if (value == nullptr) {
        value = enum_desc->FindValueByNumber(trimmed.toInt());
      }
      if (value == nullptr) {
        return false;
      }
      reflection->SetRepeatedEnum(message, field, index, value);
      return true;
    }
    default:
      return false;
  }
}

google::protobuf::Message* NavigateToMutableMessage(
    google::protobuf::Message* root,
    const std::vector<plot::MessagePathSegment>& segments,
    std::size_t end_exclusive) {
  if (root == nullptr || end_exclusive > segments.size()) {
    return nullptr;
  }
  google::protobuf::Message* current = root;
  for (std::size_t i = 0; i < end_exclusive; ++i) {
    const plot::MessagePathSegment& segment = segments[i];
    const google::protobuf::FieldDescriptor* field =
        current->GetDescriptor()->FindFieldByName(segment.field);
    if (field == nullptr) {
      return nullptr;
    }
    const google::protobuf::Reflection* reflection = current->GetReflection();
    if (field->is_repeated()) {
      if (field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE ||
          !segment.has_bracket || segment.all_elements) {
        return nullptr;
      }
      if (segment.index < 0 ||
          segment.index >= reflection->FieldSize(*current, field)) {
        return nullptr;
      }
      current = reflection->MutableRepeatedMessage(current, field, segment.index);
    } else if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      current = reflection->MutableMessage(current, field);
    } else {
      return nullptr;
    }
  }
  return current;
}

void AppendDefaultRepeatedElement(google::protobuf::Message* message,
                                  const google::protobuf::FieldDescriptor* field) {
  if (message == nullptr || field == nullptr || !field->is_repeated()) {
    return;
  }
  const google::protobuf::Reflection* reflection = message->GetReflection();
  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    reflection->AddMessage(message, field);
    return;
  }
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      reflection->AddDouble(message, field, 0.0);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      reflection->AddFloat(message, field, 0.0f);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      reflection->AddInt32(message, field, 0);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      reflection->AddInt64(message, field, 0);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      reflection->AddUInt32(message, field, 0);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      reflection->AddUInt64(message, field, 0);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      reflection->AddBool(message, field, false);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      reflection->AddString(message, field, std::string());
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      if (field->enum_type() != nullptr &&
          field->enum_type()->value_count() > 0) {
        reflection->AddEnum(message, field, field->enum_type()->value(0));
      }
      break;
    default:
      break;
  }
}

bool SetScalarAtPath(google::protobuf::Message* root, const QString& path,
                     const QString& text) {
  if (root == nullptr || path.isEmpty()) {
    return false;
  }
  const std::vector<plot::MessagePathSegment> segments =
      plot::ParseMessagePath(path.toStdString());
  if (segments.empty()) {
    return false;
  }
  const plot::MessagePathSegment& last = segments.back();
  if (last.has_bracket && !last.all_elements) {
    google::protobuf::Message* repeated_container =
        segments.size() == 1
            ? root
            : NavigateToMutableMessage(root, segments, segments.size() - 1);
    if (repeated_container == nullptr) {
      return false;
    }
    const google::protobuf::FieldDescriptor* repeated_field =
        repeated_container->GetDescriptor()->FindFieldByName(last.field);
    if (repeated_field == nullptr || !repeated_field->is_repeated() ||
        repeated_field->type() ==
            google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      return false;
    }
    return SetRepeatedScalarValue(repeated_container, repeated_field, last.index,
                                  text);
  }

  google::protobuf::Message* container =
      segments.size() == 1 ? root
                           : NavigateToMutableMessage(root, segments,
                                                      segments.size() - 1);
  if (container == nullptr) {
    return false;
  }
  const google::protobuf::FieldDescriptor* leaf =
      container->GetDescriptor()->FindFieldByName(last.field);
  if (leaf == nullptr || leaf->is_repeated() ||
      leaf->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  return SetScalarValue(container, leaf, text);
}

bool RemoveRepeatedAt(google::protobuf::Message* container,
                      const google::protobuf::FieldDescriptor* field, int index) {
  if (container == nullptr || field == nullptr || !field->is_repeated()) {
    return false;
  }
  const google::protobuf::Reflection* reflection = container->GetReflection();
  const int count = reflection->FieldSize(*container, field);
  if (index < 0 || index >= count) {
    return false;
  }
  if (index != count - 1) {
    reflection->SwapElements(container, field, index, count - 1);
  }
  reflection->RemoveLast(container, field);
  return true;
}

void DuplicateRepeatedAt(google::protobuf::Message* container,
                         const google::protobuf::FieldDescriptor* field,
                         int index) {
  if (container == nullptr || field == nullptr || !field->is_repeated()) {
    return;
  }
  const google::protobuf::Reflection* reflection = container->GetReflection();
  const int count = reflection->FieldSize(*container, field);
  if (index < 0 || index >= count) {
    return;
  }
  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    const google::protobuf::Message& source =
        reflection->GetRepeatedMessage(*container, field, index);
    google::protobuf::Message* copy = reflection->AddMessage(container, field);
    copy->CopyFrom(source);
    return;
  }
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      reflection->AddDouble(container, field,
                            reflection->GetRepeatedDouble(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      reflection->AddFloat(container, field,
                           reflection->GetRepeatedFloat(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      reflection->AddInt32(container, field,
                           reflection->GetRepeatedInt32(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      reflection->AddInt64(container, field,
                           reflection->GetRepeatedInt64(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      reflection->AddUInt32(container, field,
                            reflection->GetRepeatedUInt32(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      reflection->AddUInt64(container, field,
                            reflection->GetRepeatedUInt64(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      reflection->AddBool(container, field,
                          reflection->GetRepeatedBool(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      reflection->AddString(container, field,
                            reflection->GetRepeatedString(*container, field, index));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      reflection->AddEnum(container, field,
                          reflection->GetRepeatedEnum(*container, field, index));
      break;
    default:
      break;
  }
}

bool ResolveRepeatedFieldContainer(
    google::protobuf::Message* root, const QString& element_path,
    google::protobuf::Message** container,
    const google::protobuf::FieldDescriptor** field, int* index) {
  if (root == nullptr || container == nullptr || field == nullptr ||
      index == nullptr) {
    return false;
  }
  const std::vector<plot::MessagePathSegment> segments =
      plot::ParseMessagePath(element_path.toStdString());
  if (segments.empty()) {
    return false;
  }
  const plot::MessagePathSegment& last = segments.back();
  if (!last.has_bracket || last.all_elements) {
    return false;
  }
  *container = segments.size() == 1
                   ? root
                   : NavigateToMutableMessage(root, segments, segments.size() - 1);
  if (*container == nullptr) {
    return false;
  }
  *field = (*container)->GetDescriptor()->FindFieldByName(last.field);
  if (*field == nullptr || !(*field)->is_repeated()) {
    return false;
  }
  *index = last.index;
  return true;
}

void AppendFieldNode(QTreeWidgetItem* parent, const google::protobuf::Message& message,
                     const google::protobuf::FieldDescriptor* field,
                     const QString& path_prefix, int expr_col) {
  if (parent == nullptr || field == nullptr) {
    return;
  }
  const QString segment = ProtobufToQString(field->name());
  const QString path =
      path_prefix.isEmpty() ? segment : path_prefix + QLatin1Char('.') + segment;
  const google::protobuf::Reflection* reflection = message.GetReflection();
  const bool four_col = parent->treeWidget() != nullptr &&
                        parent->treeWidget()->columnCount() >= 4;

  if (field->is_repeated()) {
    const QString count_text =
        QStringLiteral("[%1]").arg(reflection->FieldSize(message, field));
    auto* array_item =
        four_col ? new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field),
                                                QString(), count_text})
                 : new QTreeWidgetItem(
                       parent, {segment, FieldTypeLabel(field), count_text});
    array_item->setData(0, kPathRole, path);
    array_item->setData(0, kArrayRole, true);
    array_item->setFlags(array_item->flags() & ~Qt::ItemIsEditable);
    const int count = reflection->FieldSize(message, field);
    for (int i = 0; i < count; ++i) {
      const QString index_label = QStringLiteral("[%1]").arg(i);
      const QString index_path = path + index_label;
      if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        const google::protobuf::Message& element =
            reflection->GetRepeatedMessage(message, field, i);
        auto* element_item =
            four_col ? new QTreeWidgetItem(
                           array_item,
                           {index_label, ProtobufToQString(element.GetDescriptor()->name()),
                            QString(), QString()})
                     : new QTreeWidgetItem(
                           array_item,
                           {index_label, ProtobufToQString(element.GetDescriptor()->name()),
                            QString()});
        element_item->setData(0, kPathRole, index_path);
        element_item->setData(0, kArrayIndexRole, i);
        element_item->setData(0, kElementRole, true);
        const google::protobuf::Descriptor* desc = element.GetDescriptor();
        for (int j = 0; j < desc->field_count(); ++j) {
          AppendFieldNode(element_item, element, desc->field(j), index_path, expr_col);
        }
      } else {
        auto* element_item =
            four_col ? new QTreeWidgetItem(array_item,
                                           {index_label, FieldTypeLabel(field), QString(),
                                            QString()})
                     : new QTreeWidgetItem(array_item,
                                           {index_label, FieldTypeLabel(field), QString()});
        element_item->setData(0, kPathRole, index_path);
        element_item->setData(0, kArrayIndexRole, i);
        element_item->setData(0, kElementRole, true);
        ConfigureScalarItem(element_item, field,
                            FormatRepeatedScalarValue(message, field, i), expr_col);
      }
    }
    return;
  }

  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    auto* item = four_col ? new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field),
                                                           QString(), QString()})
                          : new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field),
                                                         QString()});
    item->setData(0, kPathRole, path);
    const google::protobuf::Message& child = reflection->GetMessage(message, field);
    const google::protobuf::Descriptor* desc = child.GetDescriptor();
    for (int i = 0; i < desc->field_count(); ++i) {
      AppendFieldNode(item, child, desc->field(i), path, expr_col);
    }
    return;
  }

  const QString value =
      FieldHasValue(message, field) ? FormatScalarValue(message, field)
                                    : QStringLiteral("");
  auto* item = four_col ? new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field),
                                                       QString(), value})
                        : new QTreeWidgetItem(parent, {segment, FieldTypeLabel(field), value});
  item->setData(0, kPathRole, path);
  ConfigureScalarItem(item, field, value, expr_col);
}

}  // namespace

PublishFieldTreeWidget::PublishFieldTreeWidget(QWidget* parent)
    : QTreeWidget(parent) {
  setAlternatingRowColors(true);
  setRootIsDecorated(true);
  setUniformRowHeights(true);
  setContextMenuPolicy(Qt::CustomContextMenu);
  setDisplayMode(DisplayMode::kFieldsEditor);
  connect(this, &QTreeWidget::customContextMenuRequested, this,
          &PublishFieldTreeWidget::showContextMenu);
  connect(this, &QTreeWidget::itemChanged, this, &PublishFieldTreeWidget::onItemChanged);
  connect(this, &QTreeWidget::currentItemChanged, this,
          &PublishFieldTreeWidget::onCurrentItemChanged);
}

void PublishFieldTreeWidget::setReadOnly(bool read_only) {
  if (read_only_ == read_only) {
    return;
  }
  read_only_ = read_only;
  applyReadOnlyFlags();
}

void PublishFieldTreeWidget::setValueColumnTitle(const QString& title) {
  if (display_mode_ == DisplayMode::kFieldsEditor && headerItem() != nullptr &&
      columnCount() >= 3) {
    headerItem()->setText(expressionColumn(), title);
  }
}

void PublishFieldTreeWidget::applyReadOnlyFlags(QTreeWidgetItem* item) {
  if (item == nullptr) {
    return;
  }
  if (read_only_) {
    item->setFlags(item->flags() & ~Qt::ItemIsEditable & ~Qt::ItemIsUserCheckable);
  }
  for (int i = 0; i < item->childCount(); ++i) {
    applyReadOnlyFlags(item->child(i));
  }
}

void PublishFieldTreeWidget::applyReadOnlyFlags() {
  suppress_updates_ = true;
  for (int i = 0; i < topLevelItemCount(); ++i) {
    applyReadOnlyFlags(topLevelItem(i));
  }
  suppress_updates_ = false;
}

void PublishFieldTreeWidget::setDisplayMode(DisplayMode mode) {
  if (display_mode_ == mode && columnCount() > 0) {
    return;
  }
  display_mode_ = mode;
  if (display_mode_ == DisplayMode::kRqtPublishers) {
    setColumnCount(4);
    setHeaderLabels({tr("channel"), tr("type"), tr("rate"), tr("expression")});
    header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    header()->setSectionResizeMode(1, QHeaderView::Stretch);
    header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    header()->setStretchLastSection(true);
  } else {
    setColumnCount(3);
    setHeaderLabels({tr("Name"), tr("Type"), tr("Value")});
    header()->setStretchLastSection(true);
    header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  }
  setItemDelegateForColumn(editorValueColumn(),
                           new PublishFieldValueDelegate(this, this));
}

int PublishFieldTreeWidget::editorValueColumn() const { return expressionColumn(); }

int PublishFieldTreeWidget::expressionColumn() const {
  return display_mode_ == DisplayMode::kRqtPublishers ? 3 : 2;
}

bool PublishFieldTreeWidget::hasMessage() const { return message_ != nullptr; }

void PublishFieldTreeWidget::clearMessage() {
  suppress_updates_ = true;
  clear();
  message_.reset();
  message_type_.clear();
  suppress_updates_ = false;
}

void PublishFieldTreeWidget::setPublishers(const QVector<PublishEntry>& entries) {
  display_mode_ = DisplayMode::kRqtPublishers;
  publisher_entries_ = entries;
  publisher_messages_.clear();
  publisher_messages_.reserve(publisher_entries_.size());
  for (const PublishEntry& entry : publisher_entries_) {
    auto message = CreateMessage(entry.message_type.toStdString());
    if (message == nullptr) {
      publisher_messages_.emplace_back();
      continue;
    }
    LoadJsonIntoMessage(message.get(), entry.message_type.toStdString(),
                        entry.message_json);
    publisher_messages_.emplace_back(std::move(message));
  }
  rebuildPublishersTree();
}

QVector<PublishEntry> PublishFieldTreeWidget::publishers() const {
  QVector<PublishEntry> entries = publisher_entries_;
  for (int i = 0; i < entries.size(); ++i) {
    if (i < publisher_messages_.size() && publisher_messages_.at(i) != nullptr) {
      entries[i].message_json = MessageToJsonString(*publisher_messages_.at(i));
    }
  }
  return entries;
}

int PublishFieldTreeWidget::selectedPublisherIndex() const {
  QTreeWidgetItem* item = currentItem();
  return publisherIndexForItem(item);
}

void PublishFieldTreeWidget::setPublisherPublishing(int index, bool publishing) {
  if (index < 0 || index >= publisher_entries_.size()) {
    return;
  }
  publisher_entries_[index].publishing = publishing;
  QTreeWidgetItem* root = publisherRootItem(index);
  if (root == nullptr) {
    return;
  }
  const Qt::CheckState state = publishing ? Qt::Checked : Qt::Unchecked;
  if (root->checkState(0) == state) {
    return;
  }
  suppress_updates_ = true;
  root->setCheckState(0, state);
  suppress_updates_ = false;
}

QString PublishFieldTreeWidget::publisherJsonAt(int index) const {
  if (index < 0 || index >= publisher_entries_.size()) {
    return {};
  }
  if (index < publisher_messages_.size() &&
      publisher_messages_.at(index) != nullptr) {
    return MessageToJsonString(*publisher_messages_.at(index));
  }
  return publisher_entries_.at(index).message_json;
}

QTreeWidgetItem* PublishFieldTreeWidget::findPublisherRootItem(int index) const {
  if (index < 0) {
    return nullptr;
  }
  for (int i = 0; i < topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = topLevelItem(i);
    if (publisherIndexForItem(item) == index) {
      return item;
    }
  }
  return nullptr;
}

bool PublishFieldTreeWidget::isPublisherRootSelected(int index) const {
  QTreeWidgetItem* root = findPublisherRootItem(index);
  return root != nullptr && currentItem() == root;
}

void PublishFieldTreeWidget::selectPublisher(int index) {
  if (index < 0) {
    if (currentItem() != nullptr) {
      setCurrentItem(nullptr);
    }
    return;
  }
  QTreeWidgetItem* root = findPublisherRootItem(index);
  if (root == nullptr) {
    return;
  }
  if (currentItem() == root) {
    return;
  }
  setCurrentItem(root);
  scrollToItem(root);
}

QString PublishFieldTreeWidget::selectedPublisherId() const {
  const int index = selectedPublisherIndex();
  if (index < 0 || index >= publisher_entries_.size()) {
    return {};
  }
  return publisher_entries_.at(index).id;
}

google::protobuf::Message* PublishFieldTreeWidget::messageForItem(
    QTreeWidgetItem* item) const {
  if (item == nullptr) {
    return nullptr;
  }
  if (display_mode_ == DisplayMode::kFieldsEditor) {
    return message_.get();
  }
  QTreeWidgetItem* root = item;
  while (root != nullptr && !root->data(0, kPublisherRootRole).toBool()) {
    root = root->parent();
  }
  if (root == nullptr) {
    return nullptr;
  }
  const int index = publisherIndexForItem(item);
  if (index < 0 || index >= publisher_messages_.size()) {
    return nullptr;
  }
  return publisher_messages_.at(index).get();
}

int PublishFieldTreeWidget::publisherIndexForItem(QTreeWidgetItem* item) const {
  if (item == nullptr) {
    return -1;
  }
  QTreeWidgetItem* root = item;
  while (root != nullptr && !root->data(0, kPublisherRootRole).toBool()) {
    root = root->parent();
  }
  if (root == nullptr) {
    return -1;
  }
  const QString entry_id = root->data(0, kEntryIdRole).toString();
  for (int i = 0; i < publisher_entries_.size(); ++i) {
    if (publisher_entries_.at(i).id == entry_id) {
      return i;
    }
  }
  return -1;
}

QTreeWidgetItem* PublishFieldTreeWidget::publisherRootItem(int index) const {
  return findPublisherRootItem(index);
}

void PublishFieldTreeWidget::syncPublisherJson(int index) {
  if (index < 0 || index >= publisher_entries_.size() ||
      index >= publisher_messages_.size() ||
      publisher_messages_.at(index) == nullptr) {
    return;
  }
  publisher_entries_[index].message_json =
      MessageToJsonString(*publisher_messages_.at(index));
}

void PublishFieldTreeWidget::rebuildPublisherSubtree(int index) {
  QTreeWidgetItem* root = publisherRootItem(index);
  if (root == nullptr || index < 0 || index >= publisher_entries_.size()) {
    return;
  }
  const PublishEntry& entry = publisher_entries_.at(index);
  if (index >= publisher_messages_.size() || publisher_messages_.at(index) == nullptr) {
    return;
  }
  suppress_updates_ = true;
  while (root->childCount() > 0) {
    delete root->takeChild(0);
  }
  const google::protobuf::Message& message = *publisher_messages_.at(index);
  const google::protobuf::Descriptor* desc = message.GetDescriptor();
  for (int i = 0; i < desc->field_count(); ++i) {
    AppendFieldNode(root, message, desc->field(i), QString(), expressionColumn());
  }
  suppress_updates_ = false;
}

void PublishFieldTreeWidget::rebuildPublishersTree() {
  suppress_updates_ = true;
  clear();
  const int expr_col = expressionColumn();
  for (int index = 0; index < publisher_entries_.size(); ++index) {
    const PublishEntry& entry = publisher_entries_.at(index);
    if (index >= publisher_messages_.size() ||
        publisher_messages_.at(index) == nullptr) {
      continue;
    }
    const google::protobuf::Message& message = *publisher_messages_.at(index);
    const QString type_label =
        ShortTypeLabel(entry.message_type.toStdString());
    auto* root = new QTreeWidgetItem(
        this, {entry.channel, type_label,
               QString::number(entry.publish_rate_hz, 'f', 2), QString()});
    root->setData(0, kEntryIdRole, entry.id);
    root->setData(0, kPublisherRootRole, true);
    root->setCheckState(0, entry.publishing ? Qt::Checked : Qt::Unchecked);
    root->setFlags(root->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsEditable);
    const google::protobuf::Descriptor* desc = message.GetDescriptor();
    for (int i = 0; i < desc->field_count(); ++i) {
      AppendFieldNode(root, message, desc->field(i), QString(), expr_col);
    }
  }
  expandToDepth(1);
  suppress_updates_ = false;
}

bool PublishFieldTreeWidget::loadTemplate(const std::string& message_type) {
  message_ = CreateMessage(message_type);
  if (message_ == nullptr) {
    clearMessage();
    return false;
  }
  message_type_ = message_type;
  rebuildTree();
  return true;
}

bool PublishFieldTreeWidget::loadFromJson(const std::string& message_type,
                                          const QString& json) {
  message_ = CreateMessage(message_type);
  if (message_ == nullptr) {
    clearMessage();
    return false;
  }
  message_type_ = message_type;
  const QString trimmed = json.trimmed();
  if (trimmed.isEmpty()) {
    rebuildTree();
    return true;
  }

  google::protobuf::util::JsonParseOptions options;
  options.ignore_unknown_fields = true;
  const auto parse_status = google::protobuf::util::JsonStringToMessage(
      trimmed.toStdString(), message_.get(), options);
  if (!parse_status.ok()) {
    const CodecResult encoded =
        PublishMessageCodec::instance().encodeJson(message_type, trimmed);
    if (encoded.ok && message_->ParseFromString(encoded.payload)) {
      rebuildTree();
      return true;
    }
    message_->Clear();
  }
  rebuildTree();
  return true;
}

QString PublishFieldTreeWidget::toJson() const {
  if (message_ == nullptr || message_type_.empty()) {
    return {};
  }
  return MessageToJsonString(*message_);
}

void PublishFieldTreeWidget::rebuildTree() {
  if (display_mode_ == DisplayMode::kRqtPublishers) {
    rebuildPublishersTree();
    return;
  }
  rebuildSingleMessageTree();
}

void PublishFieldTreeWidget::rebuildSingleMessageTree() {
  suppress_updates_ = true;
  clear();
  if (message_ == nullptr) {
    suppress_updates_ = false;
    return;
  }
  const int expr_col = expressionColumn();
  const google::protobuf::Descriptor* desc = message_->GetDescriptor();
  auto* root = new QTreeWidgetItem(
      this, {ProtobufToQString(desc->name()), QStringLiteral("message"), QString()});
  root->setFlags(root->flags() & ~Qt::ItemIsEditable);
  for (int i = 0; i < desc->field_count(); ++i) {
    AppendFieldNode(root, *message_, desc->field(i), QString(), expr_col);
  }
  expandToDepth(1);
  suppress_updates_ = false;
  applyReadOnlyFlags();
}

void PublishFieldTreeWidget::onCurrentItemChanged(QTreeWidgetItem* current,
                                                  QTreeWidgetItem* /*previous*/) {
  if (display_mode_ != DisplayMode::kRqtPublishers) {
    return;
  }
  emit publisherSelectionChanged(publisherIndexForItem(current));
}

void PublishFieldTreeWidget::onItemChanged(QTreeWidgetItem* item, int column) {
  if (suppress_updates_ || read_only_ || item == nullptr) {
    return;
  }
  const int expr_col = expressionColumn();

  if (display_mode_ == DisplayMode::kRqtPublishers &&
      item->data(0, kPublisherRootRole).toBool()) {
    const int index = publisherIndexForItem(item);
    if (index < 0 || index >= publisher_entries_.size()) {
      return;
    }
    if (column == 0) {
      const bool publishing = item->checkState(0) == Qt::Checked;
      if (publisher_entries_[index].publishing != publishing) {
        publisher_entries_[index].publishing = publishing;
        emit publisherPublishingChanged(index, publishing);
      }
      return;
    }
    if (column == 1) {
      return;
    }
    if (column == 2) {
      bool ok = false;
      const double rate = item->text(2).toDouble(&ok);
      if (ok && publisher_entries_[index].publish_rate_hz != rate) {
        publisher_entries_[index].publish_rate_hz = rate;
        emit publisherRateChanged(index, rate);
      }
      return;
    }
    return;
  }

  google::protobuf::Message* message = messageForItem(item);
  if (message == nullptr) {
    return;
  }
  if (column != expr_col) {
    return;
  }
  const QString path = item->data(0, kPathRole).toString();
  if (item->data(0, kBoolRole).toBool()) {
    const QString value =
        item->checkState(expr_col) == Qt::Checked ? QStringLiteral("true")
                                                  : QStringLiteral("false");
    if (!SetScalarAtPath(message, path, value)) {
      return;
    }
    if (display_mode_ == DisplayMode::kRqtPublishers) {
      syncPublisherJson(publisherIndexForItem(item));
      emit publisherEdited(publisherIndexForItem(item));
    } else {
      emit messageEdited();
    }
    return;
  }
  if (!item->data(0, kEditableRole).toBool()) {
    return;
  }
  if (!SetScalarAtPath(message, path, item->text(expr_col))) {
    return;
  }
  if (display_mode_ == DisplayMode::kRqtPublishers) {
    syncPublisherJson(publisherIndexForItem(item));
    emit publisherEdited(publisherIndexForItem(item));
  } else {
    emit messageEdited();
  }
}

void PublishFieldTreeWidget::expandAllFields() { expandAll(); }

void PublishFieldTreeWidget::collapseAllFields() { collapseAll(); }

void PublishFieldTreeWidget::addArrayElement() {
  QTreeWidgetItem* item = currentItem();
  if (item == nullptr) {
    return;
  }
  QTreeWidgetItem* array_item = item->data(0, kArrayRole).toBool()
                                    ? item
                                    : (item->data(0, kElementRole).toBool() ? item->parent()
                                                                              : nullptr);
  if (array_item == nullptr) {
    return;
  }
  context_array_item_ = array_item;
  addRepeatedElement();
}

void PublishFieldTreeWidget::removeArrayElement() {
  QTreeWidgetItem* item = currentItem();
  if (item == nullptr) {
    return;
  }
  if (item->data(0, kElementRole).toBool()) {
    context_element_item_ = item;
    removeSelectedElement();
    return;
  }
  if (item->data(0, kArrayRole).toBool()) {
    context_array_item_ = item;
    removeRepeatedElement();
  }
}

QTreeWidgetItem* PublishFieldTreeWidget::arrayItemAt(const QPoint& pos) const {
  QTreeWidgetItem* item = itemAt(pos);
  while (item != nullptr) {
    if (item->data(0, kArrayRole).toBool()) {
      return item;
    }
    item = item->parent();
  }
  return nullptr;
}

QTreeWidgetItem* PublishFieldTreeWidget::arrayElementItemAt(const QPoint& pos) const {
  QTreeWidgetItem* item = itemAt(pos);
  if (item != nullptr && item->data(0, kElementRole).toBool()) {
    return item;
  }
  return nullptr;
}

bool PublishFieldTreeWidget::addRepeatedElementAtPath(const QString& array_path,
                                                      google::protobuf::Message* message) {
  if (message == nullptr || array_path.isEmpty()) {
    return false;
  }
  const std::vector<plot::MessagePathSegment> segments =
      plot::ParseMessagePath(array_path.toStdString());
  if (segments.empty()) {
    return false;
  }
  const plot::MessagePathSegment& last = segments.back();
  google::protobuf::Message* container =
      segments.size() == 1 ? message
                           : NavigateToMutableMessage(message, segments,
                                                      segments.size() - 1);
  if (container == nullptr) {
    return false;
  }
  const google::protobuf::FieldDescriptor* field =
      container->GetDescriptor()->FindFieldByName(last.field);
  if (field == nullptr || !field->is_repeated()) {
    return false;
  }
  AppendDefaultRepeatedElement(container, field);
  return true;
}

bool PublishFieldTreeWidget::removeRepeatedElementAtPath(
    const QString& element_path, google::protobuf::Message* message) {
  if (message == nullptr || element_path.isEmpty()) {
    return false;
  }
  google::protobuf::Message* container = nullptr;
  const google::protobuf::FieldDescriptor* field = nullptr;
  int index = -1;
  if (!ResolveRepeatedFieldContainer(message, element_path, &container, &field,
                                     &index)) {
    return false;
  }
  if (!RemoveRepeatedAt(container, field, index)) {
    return false;
  }
  return true;
}

bool PublishFieldTreeWidget::duplicateRepeatedElementAtPath(
    const QString& element_path, google::protobuf::Message* message) {
  if (message == nullptr || element_path.isEmpty()) {
    return false;
  }
  google::protobuf::Message* container = nullptr;
  const google::protobuf::FieldDescriptor* field = nullptr;
  int index = -1;
  if (!ResolveRepeatedFieldContainer(message, element_path, &container, &field,
                                     &index)) {
    return false;
  }
  DuplicateRepeatedAt(container, field, index);
  return true;
}

void PublishFieldTreeWidget::showContextMenu(const QPoint& pos) {
  QTreeWidgetItem* hit = itemAt(pos);
  google::protobuf::Message* message = messageForItem(hit);
  if (message == nullptr) {
    return;
  }
  context_message_ = message;
  QTreeWidgetItem* element_item = arrayElementItemAt(pos);
  if (element_item != nullptr) {
    context_element_item_ = element_item;
    QMenu menu(this);
    menu.addAction(tr("Duplicate element"), this,
                   &PublishFieldTreeWidget::duplicateSelectedElement);
    menu.addAction(tr("Remove element"), this,
                   &PublishFieldTreeWidget::removeSelectedElement);
    menu.exec(viewport()->mapToGlobal(pos));
    return;
  }

  QTreeWidgetItem* array_item = arrayItemAt(pos);
  if (array_item == nullptr) {
    return;
  }
  context_array_item_ = array_item;
  QMenu menu(this);
  menu.addAction(tr("Add element"), this, &PublishFieldTreeWidget::addRepeatedElement);
  const int count = array_item->childCount();
  if (count > 0) {
    menu.addAction(tr("Remove last element"), this,
                   &PublishFieldTreeWidget::removeRepeatedElement);
  }
  menu.exec(viewport()->mapToGlobal(pos));
}

void PublishFieldTreeWidget::addRepeatedElement() {
  if (context_array_item_ == nullptr) {
    return;
  }
  google::protobuf::Message* message = messageForItem(context_array_item_);
  if (message == nullptr) {
    return;
  }
  const QString array_path = context_array_item_->data(0, kPathRole).toString();
  if (!addRepeatedElementAtPath(array_path, message)) {
    return;
  }
  if (display_mode_ == DisplayMode::kRqtPublishers) {
    const int index = publisherIndexForItem(context_array_item_);
    rebuildPublisherSubtree(index);
    syncPublisherJson(index);
    emit publisherEdited(index);
  } else {
    rebuildTree();
    emit messageEdited();
  }
}

void PublishFieldTreeWidget::removeRepeatedElement() {
  if (context_array_item_ == nullptr) {
    return;
  }
  google::protobuf::Message* message = messageForItem(context_array_item_);
  if (message == nullptr) {
    return;
  }
  const int count = context_array_item_->childCount();
  if (count <= 0) {
    return;
  }
  const QString element_path =
      context_array_item_->child(count - 1)->data(0, kPathRole).toString();
  if (!removeRepeatedElementAtPath(element_path, message)) {
    return;
  }
  if (display_mode_ == DisplayMode::kRqtPublishers) {
    const int index = publisherIndexForItem(context_array_item_);
    rebuildPublisherSubtree(index);
    syncPublisherJson(index);
    emit publisherEdited(index);
  } else {
    rebuildTree();
    emit messageEdited();
  }
}

void PublishFieldTreeWidget::removeSelectedElement() {
  if (context_element_item_ == nullptr) {
    return;
  }
  google::protobuf::Message* message = messageForItem(context_element_item_);
  if (message == nullptr) {
    return;
  }
  const QString element_path = context_element_item_->data(0, kPathRole).toString();
  if (!removeRepeatedElementAtPath(element_path, message)) {
    return;
  }
  if (display_mode_ == DisplayMode::kRqtPublishers) {
    const int index = publisherIndexForItem(context_element_item_);
    rebuildPublisherSubtree(index);
    syncPublisherJson(index);
    emit publisherEdited(index);
  } else {
    rebuildTree();
    emit messageEdited();
  }
}

void PublishFieldTreeWidget::duplicateSelectedElement() {
  if (context_element_item_ == nullptr) {
    return;
  }
  google::protobuf::Message* message = messageForItem(context_element_item_);
  if (message == nullptr) {
    return;
  }
  const QString element_path = context_element_item_->data(0, kPathRole).toString();
  if (!duplicateRepeatedElementAtPath(element_path, message)) {
    return;
  }
  if (display_mode_ == DisplayMode::kRqtPublishers) {
    const int index = publisherIndexForItem(context_element_item_);
    rebuildPublisherSubtree(index);
    syncPublisherJson(index);
    emit publisherEdited(index);
  } else {
    rebuildTree();
    emit messageEdited();
  }
}

}  // namespace publish_panel
}  // namespace autoviz
