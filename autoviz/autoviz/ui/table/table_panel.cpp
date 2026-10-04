/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_panel.hpp"

#include <algorithm>

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <QAbstractItemView>
#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFocusEvent>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMimeData>
#include <QTableWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/protobuf_qt_string.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/plot/message_path_navigation.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_path_utils.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace table_panel {
namespace {

using automsgs::msgs::DynamicFactory;

constexpr int kMaxRows = 2000;

std::string StripPackagePrefix(const std::string& type_name) {
  static const char* kPrefix = "automsgs.msgs.";
  if (type_name.rfind(kPrefix, 0) == 0) {
    return type_name.substr(std::char_traits<char>::length(kPrefix));
  }
  return type_name;
}

google::protobuf::Message* ParseMessageWithGeneratedPool(
    const std::string& normalized, const std::string& payload,
    DynamicFactory::MessagePtr* out) {
  if (out == nullptr || normalized.empty() || payload.empty()) {
    return nullptr;
  }
  const google::protobuf::DescriptorPool* pool =
      google::protobuf::DescriptorPool::generated_pool();
  if (pool == nullptr) {
    return nullptr;
  }
  const google::protobuf::Descriptor* desc =
      pool->FindMessageTypeByName(normalized);
  if (desc == nullptr) {
    desc = pool->FindMessageTypeByName(StripPackagePrefix(normalized));
  }
  if (desc == nullptr) {
    return nullptr;
  }
  const google::protobuf::Message* prototype =
      google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc);
  if (prototype == nullptr) {
    return nullptr;
  }
  *out = DynamicFactory::MessagePtr(prototype->New());
  if (*out == nullptr || !(*out)->ParseFromString(payload)) {
    out->reset();
    return nullptr;
  }
  return out->get();
}

google::protobuf::Message* ParsePayload(const std::string& message_type,
                                        const std::string& payload,
                                        DynamicFactory::MessagePtr* out) {
  if (out == nullptr || message_type.empty() || payload.empty()) {
    return nullptr;
  }
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  const std::string decoded = integration::DecodeChannelPayload(payload);

  auto try_parse = [&](const std::string& bytes) -> google::protobuf::Message* {
    if (google::protobuf::Message* parsed =
            ParseMessageWithGeneratedPool(normalized, bytes, out)) {
      return parsed;
    }
    static DynamicFactory factory;
    *out = factory.New(normalized);
    if (*out == nullptr) {
      *out = factory.New(StripPackagePrefix(normalized));
    }
    if (*out != nullptr && (*out)->ParseFromString(bytes)) {
      return out->get();
    }
    out->reset();
    return nullptr;
  };

  if (try_parse(decoded.empty() ? payload : decoded) != nullptr) {
    return out->get();
  }
  if (!decoded.empty() && decoded != payload && try_parse(payload) != nullptr) {
    return out->get();
  }
  return nullptr;
}

QString FormatScalar(const google::protobuf::Message& message,
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
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
      const std::string value = reflection->GetString(message, field);
      if (value.size() > 80) {
        return QStringLiteral("%1…")
            .arg(QString::fromStdString(value.substr(0, 77)));
      }
      return QString::fromStdString(value);
    }
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const google::protobuf::EnumValueDescriptor* ev =
          reflection->GetEnum(message, field);
      return ev != nullptr ? ProtobufToQString(ev->name())
                           : QStringLiteral("(enum)");
    }
    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      return QString::fromStdString(
          reflection->GetMessage(message, field).ShortDebugString());
    default:
      return QStringLiteral("—");
  }
}

QString FormatRepeatedScalar(const google::protobuf::Message& message,
                             const google::protobuf::FieldDescriptor* field,
                             int index) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return QString::number(reflection->GetRepeatedDouble(message, field, index),
                             'g', 8);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return QString::number(reflection->GetRepeatedFloat(message, field, index),
                             'g', 6);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return QString::number(
          reflection->GetRepeatedInt32(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return QString::number(
          reflection->GetRepeatedInt64(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return QString::number(
          reflection->GetRepeatedUInt32(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return QString::number(
          reflection->GetRepeatedUInt64(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetRepeatedBool(message, field, index)
                 ? QStringLiteral("true")
                 : QStringLiteral("false");
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return QString::fromStdString(
          reflection->GetRepeatedString(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const google::protobuf::EnumValueDescriptor* ev =
          reflection->GetRepeatedEnum(message, field, index);
      return ev != nullptr ? ProtobufToQString(ev->name())
                           : QStringLiteral("(enum)");
    }
    default:
      return QStringLiteral("—");
  }
}

/** Strip trailing [:] / [] so ResolveRepeatedFieldPath sees a plain field. */
QString NormalizeArrayFieldPath(QString path) {
  path = path.trimmed();
  if (path.endsWith(QLatin1String("[:]"))) {
    path.chop(3);
  } else if (path.endsWith(QLatin1String("[]"))) {
    path.chop(2);
  }
  while (path.endsWith(QLatin1Char('.'))) {
    path.chop(1);
  }
  return path;
}

bool AcceptsTableMime(const QMimeData* mime) {
  return mime != nullptr &&
         (mime->hasFormat(QLatin1String(plot::kPlotSeriesDragMime)) ||
          mime->hasFormat(QLatin1String(plot::kPlotSeriesListDragMime)));
}

}  // namespace

TablePanel::TablePanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager),
      config_(DefaultTablePanelConfig()),
      QWidget(parent) {
  ApplyPanelShell(this);
  setObjectName(QStringLiteral("TablePanelContent"));
  setAcceptDrops(true);
  applyChromeStyles();

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto* toolbar = new QWidget(this);
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(10, 8, 10, 8);
  toolbar_layout->setSpacing(8);

  auto* path_label = new QLabel(tr("Path"), toolbar);
  path_label->setStyleSheet(style::type(style::Role::PanelMuted, 11));
  toolbar_layout->addWidget(path_label);

  path_edit_ = new QLineEdit(toolbar);
  path_edit_->setObjectName(QStringLiteral("AutovizCyanLineEdit"));
  path_edit_->setPlaceholderText(tr("/channel.array_field"));
  path_edit_->setClearButtonEnabled(true);
  path_edit_->setStyleSheet(
      style::sheet(QStringLiteral("widget/fields"), style::Tone::Frost));
  toolbar_layout->addWidget(path_edit_, 1);

  row_filter_edit_ = new QLineEdit(toolbar);
  row_filter_edit_->setObjectName(QStringLiteral("AutovizCyanLineEdit"));
  row_filter_edit_->setPlaceholderText(tr("Filter rows"));
  row_filter_edit_->setClearButtonEnabled(true);
  row_filter_edit_->setStyleSheet(
      style::sheet(QStringLiteral("widget/fields"), style::Tone::Frost));
  toolbar_layout->addWidget(row_filter_edit_);

  status_label_ = new QLabel(toolbar);
  status_label_->setObjectName(QStringLiteral("AutovizCyanStatusChip"));
  status_label_->setStyleSheet(
      style::sheet(QStringLiteral("widget/badges"), style::Tone::Frost));
  toolbar_layout->addWidget(status_label_);
  layout->addWidget(toolbar);

  table_ = new QTableWidget(this);
  table_->setObjectName(QStringLiteral("TablePanelGrid"));
  table_->setAlternatingRowColors(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSortingEnabled(true);
  table_->verticalHeader()->setVisible(true);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
  table_->setStyleSheet(
      style::sheet(QStringLiteral("channels"), style::Tone::Frost));
  table_->setContextMenuPolicy(Qt::CustomContextMenu);
  layout->addWidget(table_, 1);

  connect(path_edit_, &QLineEdit::editingFinished, this,
          &TablePanel::onPathEdited);
  connect(row_filter_edit_, &QLineEdit::textChanged, this,
          &TablePanel::onRowFilterEdited);
  connect(table_, &QTableWidget::customContextMenuRequested, this,
          &TablePanel::onTableContextMenu);

  tick_timer_ = new QTimer(this);
  tick_timer_->setInterval(50);
  connect(tick_timer_, &QTimer::timeout, this, &TablePanel::onTick);
  tick_timer_->start();

  clearTable(tr("Drop an array field from Channels, or type /channel.path"));
}

TablePanel::~TablePanel() { unsubscribe(); }

void TablePanel::applyChromeStyles() {
  setStyleSheet(style::sheet(QStringLiteral("channels"), style::Tone::Frost));
}

TablePanelConfig TablePanel::config() const { return config_; }

void TablePanel::setConfig(const TablePanelConfig& config) {
  config_ = config;
  if (config_.title.trimmed().isEmpty()) {
    config_.title = QStringLiteral("Table");
  }
  const QString combined =
      plot::CombinedPlotValuePath(config_.channel, config_.field_path);
  if (path_edit_ != nullptr) {
    path_edit_->blockSignals(true);
    path_edit_->setText(combined);
    path_edit_->blockSignals(false);
  }
  if (row_filter_edit_ != nullptr) {
    row_filter_edit_->blockSignals(true);
    row_filter_edit_->setText(config_.row_filter);
    row_filter_edit_->blockSignals(false);
  }
  resubscribe();
  emitConfigChanged();
}

void TablePanel::cloneConfigFrom(const TablePanelConfig& config) {
  setConfig(config);
}

void TablePanel::setSource(const QString& channel, const QString& field_path) {
  config_.channel = channel.trimmed();
  config_.field_path = NormalizeArrayFieldPath(field_path);
  if (config_.title.trimmed().isEmpty() ||
      config_.title == QLatin1String("Table")) {
    config_.title = config_.channel.isEmpty()
                        ? QStringLiteral("Table")
                        : config_.channel;
  }
  const QString combined =
      plot::CombinedPlotValuePath(config_.channel, config_.field_path);
  if (path_edit_ != nullptr) {
    path_edit_->blockSignals(true);
    path_edit_->setText(combined);
    path_edit_->blockSignals(false);
  }
  resubscribe();
  emitConfigChanged();
}

void TablePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("TableDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreatePanelTitleBarTools(dock, callbacks, options);
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

void TablePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

void TablePanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void TablePanel::dragEnterEvent(QDragEnterEvent* event) {
  if (AcceptsTableMime(event->mimeData())) {
    event->acceptProposedAction();
    return;
  }
  QWidget::dragEnterEvent(event);
}

void TablePanel::dragMoveEvent(QDragMoveEvent* event) {
  if (AcceptsTableMime(event->mimeData())) {
    event->acceptProposedAction();
    return;
  }
  QWidget::dragMoveEvent(event);
}

void TablePanel::dropEvent(QDropEvent* event) {
  const QVector<plot::PlotSeriesDragPayload> payloads =
      plot::ReadPlotSeriesDragPayloads(event->mimeData());
  if (payloads.isEmpty()) {
    QWidget::dropEvent(event);
    return;
  }
  const plot::PlotSeriesDragPayload& first = payloads.first();
  setSource(first.channel, first.field_path);
  event->acceptProposedAction();
}

void TablePanel::onPathEdited() {
  if (path_edit_ == nullptr) {
    return;
  }
  QString channel;
  QString field_path;
  plot::SplitPlotValuePath(path_edit_->text().trimmed(),
                           plot::AllKnownChannels(manager_), &channel,
                           &field_path);
  setSource(channel, field_path);
}

void TablePanel::onTick() {
  std::optional<std::string> latest = payload_queue_.takeLatest();
  if (!latest.has_value()) {
    return;
  }
  renderPayload(*latest);
}

void TablePanel::unsubscribe() {
  if (subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
    subscription_id_ = 0;
  }
  active_channel_.clear();
  active_message_type_.clear();
}

void TablePanel::resubscribe() {
  unsubscribe();
  payload_queue_ = integration::MessageQueue{};
  if (config_.channel.isEmpty() || config_.field_path.isEmpty()) {
    clearTable(tr("Set a channel and array field path"));
    return;
  }
  active_channel_ = config_.channel.toStdString();
  active_message_type_ = messageTypeForChannel(config_.channel);
  if (active_message_type_.empty()) {
    clearTable(tr("Unknown channel schema"));
    return;
  }
  subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      active_channel_, [this](const std::string& payload) {
        payload_queue_.push(payload);
      });
  updateStatus(tr("Waiting for %1…")
                   .arg(plot::CombinedPlotValuePath(config_.channel,
                                                    config_.field_path)));
}

std::string TablePanel::messageTypeForChannel(const QString& channel) const {
  return plot::MessageTypeForChannel(manager_, channel);
}

void TablePanel::clearTable(const QString& status_hint) {
  if (table_ != nullptr) {
    table_->clear();
    table_->setRowCount(0);
    table_->setColumnCount(0);
  }
  updateStatus(status_hint);
}

void TablePanel::updateStatus(const QString& text) {
  if (status_label_ != nullptr) {
    status_label_->setText(text);
  }
}

void TablePanel::onRowFilterEdited(const QString& text) {
  if (config_.row_filter == text) {
    return;
  }
  config_.row_filter = text;
  applyRowFilter();
  emitConfigChanged();
}

void TablePanel::applyRowFilter() {
  if (table_ == nullptr) {
    return;
  }
  const QString filter = config_.row_filter.trimmed();
  const int rows = table_->rowCount();
  int shown = 0;
  for (int row = 0; row < rows; ++row) {
    bool match = filter.isEmpty();
    if (!match) {
      for (int column = 0; column < table_->columnCount(); ++column) {
        const QTableWidgetItem* item = table_->item(row, column);
        if (item != nullptr &&
            item->text().contains(filter, Qt::CaseInsensitive)) {
          match = true;
          break;
        }
      }
    }
    table_->setRowHidden(row, !match);
    if (match) {
      ++shown;
    }
  }
  if (rows <= 0 || filter.isEmpty()) {
    return;
  }
  updateStatus(tr("%1 / %2 rows").arg(shown).arg(rows));
}

void TablePanel::onTableContextMenu(const QPoint& pos) {
  if (table_ == nullptr) {
    return;
  }
  const QTableWidgetItem* cell = table_->itemAt(pos);
  QMenu menu(this);
  QAction* copy_cell = nullptr;
  if (cell != nullptr && !cell->text().isEmpty()) {
    copy_cell = menu.addAction(tr("Copy cell"));
  }
  QAction* copy_row = nullptr;
  if (cell != nullptr) {
    copy_row = menu.addAction(tr("Copy row"));
  }
  const QList<QTableWidgetItem*> selected = table_->selectedItems();
  QAction* copy_selection = nullptr;
  if (!selected.isEmpty()) {
    copy_selection = menu.addAction(tr("Copy selection"));
  }
  if (menu.isEmpty()) {
    return;
  }
  QAction* chosen = menu.exec(table_->viewport()->mapToGlobal(pos));
  QClipboard* clipboard = QApplication::clipboard();
  if (chosen == nullptr || clipboard == nullptr) {
    return;
  }
  if (chosen == copy_cell) {
    clipboard->setText(cell->text());
    return;
  }

  auto row_text = [this](int row) {
    QStringList cells;
    for (int column = 0; column < table_->columnCount(); ++column) {
      const QTableWidgetItem* item = table_->item(row, column);
      cells.push_back(item != nullptr ? item->text() : QString());
    }
    return cells.join(QLatin1Char('\t'));
  };

  if (chosen == copy_row) {
    clipboard->setText(row_text(cell->row()));
    return;
  }
  if (chosen == copy_selection) {
    QList<int> rows;
    for (const QTableWidgetItem* item : selected) {
      if (item == nullptr || rows.contains(item->row())) {
        continue;
      }
      rows.push_back(item->row());
    }
    std::sort(rows.begin(), rows.end());
    QStringList lines;
    QStringList headers;
    for (int column = 0; column < table_->columnCount(); ++column) {
      const QTableWidgetItem* header = table_->horizontalHeaderItem(column);
      headers.push_back(header != nullptr ? header->text() : QString());
    }
    if (!headers.isEmpty()) {
      lines.push_back(headers.join(QLatin1Char('\t')));
    }
    for (int row : rows) {
      lines.push_back(row_text(row));
    }
    clipboard->setText(lines.join(QLatin1Char('\n')));
  }
}

void TablePanel::emitConfigChanged() { emit configChanged(); }

void TablePanel::renderPayload(const std::string& payload) {
  DynamicFactory::MessagePtr holder;
  google::protobuf::Message* message =
      ParsePayload(active_message_type_, payload, &holder);
  if (message == nullptr) {
    updateStatus(tr("Parse failed"));
    return;
  }

  plot::ResolvedRepeatedField repeated;
  if (!plot::ResolveRepeatedFieldPath(*message, config_.field_path.toStdString(),
                                      &repeated) ||
      repeated.container == nullptr || repeated.repeated_field == nullptr) {
    clearTable(tr("Path is not a repeated field: %1").arg(config_.field_path));
    return;
  }

  const google::protobuf::Reflection* reflection =
      repeated.container->GetReflection();
  const google::protobuf::FieldDescriptor* field = repeated.repeated_field;
  const int count = reflection->FieldSize(*repeated.container, field);
  if (count <= 0) {
    clearTable(tr("Empty array (%1)").arg(config_.field_path));
    return;
  }

  const bool sorting = table_->isSortingEnabled();
  table_->setSortingEnabled(false);

  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    const google::protobuf::Descriptor* elem_desc = field->message_type();
    QStringList headers;
    QVector<const google::protobuf::FieldDescriptor*> columns;
    if (elem_desc != nullptr) {
      for (int i = 0; i < elem_desc->field_count(); ++i) {
        const google::protobuf::FieldDescriptor* child = elem_desc->field(i);
        if (child == nullptr || child->is_repeated()) {
          continue;
        }
        // One-level flatten: scalars + enums + strings; nested messages as debug.
        headers.push_back(ProtobufToQString(child->name()));
        columns.push_back(child);
      }
    }
    if (headers.isEmpty()) {
      headers.push_back(QStringLiteral("value"));
    }

    table_->setColumnCount(headers.size());
    table_->setHorizontalHeaderLabels(headers);

    int rows = 0;
    table_->setRowCount(0);
    for (int i = 0; i < count && rows < kMaxRows; ++i) {
      if (repeated.use_single_index && i != repeated.single_index) {
        continue;
      }
      const google::protobuf::Message& element =
          reflection->GetRepeatedMessage(*repeated.container, field, i);
      if (repeated.element_filter.has_value() &&
          !plot::ElementMatchesPathFilter(element, *repeated.element_filter)) {
        continue;
      }
      const int row = table_->rowCount();
      table_->insertRow(row);
      if (columns.isEmpty()) {
        table_->setItem(
            row, 0,
            new QTableWidgetItem(QString::fromStdString(element.ShortDebugString())));
      } else {
        for (int c = 0; c < columns.size(); ++c) {
          table_->setItem(row, c,
                          new QTableWidgetItem(FormatScalar(element, columns.at(c))));
        }
      }
      ++rows;
    }
    updateStatus(rows == 1 ? tr("1 row")
                           : tr("%1 rows%2")
                                 .arg(rows)
                                 .arg(count > kMaxRows ? tr(" (capped)") : QString()));
  } else {
    table_->setColumnCount(1);
    table_->setHorizontalHeaderLabels({ProtobufToQString(field->name())});
    table_->setRowCount(0);
    int rows = 0;
    for (int i = 0; i < count && rows < kMaxRows; ++i) {
      if (repeated.use_single_index && i != repeated.single_index) {
        continue;
      }
      const int row = table_->rowCount();
      table_->insertRow(row);
      table_->setItem(
          row, 0,
          new QTableWidgetItem(
              FormatRepeatedScalar(*repeated.container, field, i)));
      ++rows;
    }
    updateStatus(rows == 1 ? tr("1 row") : tr("%1 rows").arg(rows));
  }

  table_->setSortingEnabled(sorting);
  applyRowFilter();
}

}  // namespace table_panel
}  // namespace autoviz
