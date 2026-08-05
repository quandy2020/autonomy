/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/parameters/parameters_panel.hpp"

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>

#include <QCheckBox>
#include <QComboBox>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QShowEvent>
#include <QTableWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autolink/parameter/parameter_client.hpp"
#include "autolink/proto/parameter.pb.h"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/parameter_node_discovery.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/parameters/parameter_value_format.hpp"

namespace autoviz {
namespace parameters_panel {
namespace {

constexpr int kColumnName = 0;
constexpr int kColumnType = 1;
constexpr int kColumnValue = 2;

QByteArray EncodeParameterRow(const autolink::Parameter& parameter) {
  return QByteArray::fromStdString(parameter.ToProtoParam().SerializeAsString());
}

std::optional<autolink::Parameter> DecodeParameterRow(const QTableWidgetItem* item) {
  if (item == nullptr) {
    return std::nullopt;
  }
  const QByteArray bytes = item->data(Qt::UserRole).toByteArray();
  if (bytes.isEmpty()) {
    return std::nullopt;
  }
  autolink::proto::Param proto;
  if (!proto.ParseFromArray(bytes.constData(), static_cast<int>(bytes.size()))) {
    return std::nullopt;
  }
  autolink::Parameter parameter;
  parameter.FromProtoParam(proto);
  return std::optional<autolink::Parameter>(autolink::Parameter(parameter));
}

void SetRowData(QTableWidgetItem* item, const autolink::Parameter& parameter) {
  if (item == nullptr) {
    return;
  }
  item->setData(Qt::UserRole, EncodeParameterRow(parameter));
}

bool RowMatchesFilter(QTableWidget* table, int row, const QString& needle) {
  if (needle.isEmpty()) {
    return true;
  }
  for (int column = kColumnName; column <= kColumnValue; ++column) {
    if (QTableWidgetItem* item = table->item(row, column)) {
      if (item->text().contains(needle, Qt::CaseInsensitive)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

ParametersPanelConfig DefaultParametersPanelConfig() {
  ParametersPanelConfig config;
  config.auto_refresh = true;
  return config;
}

ParametersPanel::ParametersPanel(common::VisualizationManager* manager,
                                 QWidget* parent)
    : manager_(manager), config_(DefaultParametersPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setupUi();
  applyConfigToUi();
  refreshNodeList();
}

void ParametersPanel::setupUi() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* toolbar = new QFrame(this);
  toolbar->setStyleSheet(PanelStatusBarStyle());
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(8, 6, 8, 6);
  toolbar_layout->setSpacing(6);

  toolbar_layout->addWidget(new QLabel(tr("Node"), toolbar));
  node_combo_ = new QComboBox(toolbar);
  node_combo_->setMinimumWidth(160);
  node_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  toolbar_layout->addWidget(node_combo_, 1);

  filter_edit_ = new QLineEdit(toolbar);
  filter_edit_->setPlaceholderText(tr("Filter parameters…"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setMinimumWidth(140);
  toolbar_layout->addWidget(filter_edit_, 1);

  refresh_button_ = new QToolButton(toolbar);
  refresh_button_->setToolTip(tr("Refresh parameters"));
  refresh_button_->setIcon(
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_reset_view.svg")));
  toolbar_layout->addWidget(refresh_button_);

  auto_refresh_check_ = new QCheckBox(tr("Auto"), toolbar);
  auto_refresh_check_->setToolTip(tr("Automatically refresh parameters"));
  auto_refresh_check_->setChecked(true);
  toolbar_layout->addWidget(auto_refresh_check_);

  root->addWidget(toolbar);

  table_ = new QTableWidget(this);
  table_->setColumnCount(3);
  table_->setHorizontalHeaderLabels({tr("Name"), tr("Type"), tr("Value")});
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->horizontalHeader()->setSectionResizeMode(kColumnName,
                                                   QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(kColumnType,
                                                   QHeaderView::ResizeToContents);
  table_->verticalHeader()->setVisible(false);
  table_->setAlternatingRowColors(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setEditTriggers(QAbstractItemView::DoubleClicked |
                          QAbstractItemView::SelectedClicked |
                          QAbstractItemView::EditKeyPressed);
  table_->setWordWrap(false);
  root->addWidget(table_, 1);

  auto* status_bar = new QFrame(this);
  status_bar->setStyleSheet(PanelFooterStyle());
  auto* status_layout = new QHBoxLayout(status_bar);
  status_layout->setContentsMargins(8, 4, 8, 4);
  status_label_ = new QLabel(status_bar);
  status_label_->setStyleSheet(PanelStatusLabelStyle());
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  status_layout->addWidget(status_label_, 1);
  root->addWidget(status_bar);

  connect(node_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &ParametersPanel::onNodeSelectionChanged);
  connect(filter_edit_, &QLineEdit::textChanged, this,
          &ParametersPanel::onFilterChanged);
  connect(auto_refresh_check_, &QCheckBox::toggled, this,
          &ParametersPanel::onAutoRefreshToggled);
  connect(refresh_button_, &QToolButton::clicked, this,
          &ParametersPanel::onRefreshClicked);
  connect(table_, &QTableWidget::itemChanged, this,
          &ParametersPanel::onParameterItemChanged);

  node_timer_ = new QTimer(this);
  node_timer_->setInterval(2000);
  connect(node_timer_, &QTimer::timeout, this, &ParametersPanel::refreshNodeList);

  parameter_timer_ = new QTimer(this);
  parameter_timer_->setInterval(2000);
  connect(parameter_timer_, &QTimer::timeout, this,
          &ParametersPanel::refreshParameters);

  filter_timer_ = new QTimer(this);
  filter_timer_->setSingleShot(true);
  filter_timer_->setInterval(150);
  connect(filter_timer_, &QTimer::timeout, this, [this]() { applyFilter(); });

  updateStatusText(tr("Select a node to view runtime parameters."));
}

void ParametersPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  auto* tools = new QWidget(dock);
  tools->setStyleSheet(PlotTitleToolsStyleSheet());
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 4, 0);
  layout->setSpacing(0);
  layout->addWidget(CreateTitleSeparator(tools));

  auto* split_right = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_right.svg")),
      tr("Split right"));
  layout->addWidget(split_right);
  connect(split_right, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Horizontal);
  });

  auto* split_down = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_down.svg")),
      tr("Split down"));
  layout->addWidget(split_down);
  connect(split_down, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Vertical);
  });

  auto* expand = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_fullscreen.svg")),
      tr("Expand"), true);
  expand_button_ = expand;
  layout->addWidget(expand);
  connect(expand, &QToolButton::clicked, this, [this]() { emit panelExpandRequested(); });

  auto* more_button = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("ParametersDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };
  more_button->setMenu(CreatePanelContextMenu(more_button, callbacks));
  layout->addWidget(more_button);

  dock->setTitleBarTools(tools);
}

void ParametersPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->setChecked(checked);
  }
}

ParametersPanelConfig ParametersPanel::config() const { return config_; }

void ParametersPanel::setConfig(const ParametersPanelConfig& config) {
  config_ = config;
  applyConfigToUi();
}

void ParametersPanel::cloneConfigFrom(const ParametersPanelConfig& config) {
  setConfig(config);
}

void ParametersPanel::applyConfigToUi() {
  if (filter_edit_ != nullptr) {
    filter_edit_->blockSignals(true);
    filter_edit_->setText(config_.filter);
    filter_edit_->blockSignals(false);
  }
  if (auto_refresh_check_ != nullptr) {
    auto_refresh_check_->setChecked(config_.auto_refresh);
  }
  if (!config_.service_node.isEmpty() && node_combo_ != nullptr) {
    const int index = node_combo_->findData(config_.service_node);
    if (index >= 0) {
      node_combo_->setCurrentIndex(index);
    }
  }
  applyFilter();
}

void ParametersPanel::syncConfigFromUi() {
  if (node_combo_ != nullptr) {
    config_.service_node = node_combo_->currentData().toString();
  }
  if (filter_edit_ != nullptr) {
    config_.filter = filter_edit_->text();
  }
  if (auto_refresh_check_ != nullptr) {
    config_.auto_refresh = auto_refresh_check_->isChecked();
  }
}

void ParametersPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void ParametersPanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (node_timer_ != nullptr) {
    node_timer_->start();
  }
  if (parameter_timer_ != nullptr && config_.auto_refresh) {
    parameter_timer_->start();
  }
  refreshNodeList();
  refreshParameters();
}

void ParametersPanel::hideEvent(QHideEvent* event) {
  if (node_timer_ != nullptr) {
    node_timer_->stop();
  }
  if (parameter_timer_ != nullptr) {
    parameter_timer_->stop();
  }
  QWidget::hideEvent(event);
}

void ParametersPanel::refreshNodeList() {
  if (node_combo_ == nullptr) {
    return;
  }
  const QString previous = node_combo_->currentData().toString();
  populateNodeCombo(integration::ListParameterServerNodes());
  if (!previous.isEmpty()) {
    const int restore_index = node_combo_->findData(previous);
    if (restore_index >= 0) {
      node_combo_->setCurrentIndex(restore_index);
    }
  } else if (!config_.service_node.isEmpty()) {
    const int restore_index = node_combo_->findData(config_.service_node);
    if (restore_index >= 0) {
      node_combo_->setCurrentIndex(restore_index);
    }
  }
}

void ParametersPanel::populateNodeCombo(const std::vector<std::string>& nodes) {
  node_combo_->blockSignals(true);
  const QString previous = node_combo_->currentData().toString();
  node_combo_->clear();
  node_combo_->addItem(tr("(select node)"), QString());
  for (const std::string& node : nodes) {
    const QString node_q = QString::fromStdString(node);
    node_combo_->addItem(node_q, node_q);
  }
  const int restore_index = node_combo_->findData(previous);
  if (restore_index >= 0) {
    node_combo_->setCurrentIndex(restore_index);
  }
  node_combo_->blockSignals(false);
}

void ParametersPanel::onNodeSelectionChanged(int index) {
  syncConfigFromUi();
  emit configChanged();
  if (index <= 0) {
    suppress_table_signals_ = true;
    table_->setRowCount(0);
    suppress_table_signals_ = false;
    updateStatusText(tr("Select a node to view runtime parameters."));
    return;
  }
  refreshParameters();
}

void ParametersPanel::onFilterChanged(const QString& text) {
  config_.filter = text;
  emit configChanged();
  if (filter_timer_ != nullptr) {
    filter_timer_->start();
  }
}

void ParametersPanel::onAutoRefreshToggled(bool enabled) {
  config_.auto_refresh = enabled;
  emit configChanged();
  if (parameter_timer_ == nullptr) {
    return;
  }
  if (enabled && isVisible()) {
    parameter_timer_->start();
  } else {
    parameter_timer_->stop();
  }
}

void ParametersPanel::onRefreshClicked() { refreshParameters(); }

void ParametersPanel::refreshParameters() {
  if (loading_parameters_ || node_combo_ == nullptr || table_ == nullptr) {
    return;
  }
  const QString service_node = node_combo_->currentData().toString();
  if (service_node.isEmpty()) {
    return;
  }

  loading_parameters_ = true;
  std::vector<autolink::Parameter> parameters;
  QString error_message;
  const bool ok = loadParametersForCurrentNode(&parameters, &error_message);
  loading_parameters_ = false;
  if (!ok) {
    updateStatusText(error_message, true);
    return;
  }

  rebuildParameterTable(parameters);
  applyFilter();
  updateStatusText(tr("%1 parameters from %2")
                       .arg(table_->rowCount())
                       .arg(service_node));
}

bool ParametersPanel::loadParametersForCurrentNode(
    std::vector<autolink::Parameter>* parameters, QString* error_message) {
  if (parameters == nullptr || manager_ == nullptr || node_combo_ == nullptr) {
    return false;
  }
  parameters->clear();

  const std::shared_ptr<autolink::Node> node = manager_->autolinkNode();
  if (node == nullptr) {
    if (error_message != nullptr) {
      *error_message = tr("Autolink node is not available.");
    }
    return false;
  }

  const std::string service_node = node_combo_->currentData().toString().toStdString();
  if (service_node.empty()) {
    if (error_message != nullptr) {
      *error_message = tr("Select a parameter server node.");
    }
    return false;
  }

  autolink::ParameterClient client(node, service_node);
  if (!client.ListParameters(parameters)) {
    if (error_message != nullptr) {
      *error_message =
          tr("Failed to list parameters for %1.")
              .arg(QString::fromStdString(service_node));
    }
    return false;
  }

  std::vector<std::size_t> order(parameters->size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&parameters](std::size_t left, std::size_t right) {
              return (*parameters)[left].Name() < (*parameters)[right].Name();
            });
  std::vector<autolink::Parameter> sorted;
  sorted.reserve(order.size());
  for (std::size_t index : order) {
    sorted.push_back(autolink::Parameter((*parameters)[index]));
  }
  *parameters = std::move(sorted);
  return true;
}

void ParametersPanel::rebuildParameterTable(
    const std::vector<autolink::Parameter>& parameters) {
  suppress_table_signals_ = true;
  table_->setRowCount(0);
  table_->setRowCount(static_cast<int>(parameters.size()));

  for (int row = 0; row < static_cast<int>(parameters.size()); ++row) {
    const autolink::Parameter& parameter = parameters[static_cast<std::size_t>(row)];

    auto* name_item = new QTableWidgetItem(QString::fromStdString(parameter.Name()));
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    SetRowData(name_item, parameter);
    table_->setItem(row, kColumnName, name_item);

    auto* type_item =
        new QTableWidgetItem(FormatParameterTypeName(parameter));
    type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
    table_->setItem(row, kColumnType, type_item);

    auto* value_item = new QTableWidgetItem(FormatParameterValue(parameter));
    Qt::ItemFlags value_flags = value_item->flags() | Qt::ItemIsSelectable |
                                Qt::ItemIsEnabled;
    if (IsEditableParameterType(parameter.Type())) {
      value_flags |= Qt::ItemIsEditable;
    } else {
      value_flags &= ~Qt::ItemIsEditable;
      value_item->setToolTip(tr("Protobuf parameters are read-only in Autoviz."));
    }
    value_item->setFlags(value_flags);
    table_->setItem(row, kColumnValue, value_item);
  }

  suppress_table_signals_ = false;
}

void ParametersPanel::applyFilter() {
  if (table_ == nullptr || filter_edit_ == nullptr) {
    return;
  }
  const QString needle = filter_edit_->text().trimmed();
  for (int row = 0; row < table_->rowCount(); ++row) {
    table_->setRowHidden(row, !RowMatchesFilter(table_, row, needle));
  }
}

void ParametersPanel::updateStatusText(const QString& text, bool is_error) {
  if (status_label_ == nullptr) {
    return;
  }
  status_label_->setText(text);
  status_label_->setStyleSheet(
      is_error ? QStringLiteral("color: palette(negative); font-size: 10px;")
               : PanelStatusLabelStyle());
}

bool ParametersPanel::commitParameterEdit(int row, const QString& new_text) {
  QTableWidgetItem* name_item = table_->item(row, kColumnName);
  const std::optional<autolink::Parameter> current = DecodeParameterRow(name_item);
  if (!current.has_value() || manager_ == nullptr || node_combo_ == nullptr) {
    return false;
  }

  QString parse_error;
  const std::optional<autolink::Parameter> parsed =
      ParseEditedParameterValue(*current, new_text, &parse_error);
  if (!parsed.has_value()) {
    updateStatusText(parse_error, true);
    return false;
  }

  const std::shared_ptr<autolink::Node> node = manager_->autolinkNode();
  if (node == nullptr) {
    updateStatusText(tr("Autolink node is not available."), true);
    return false;
  }

  const std::string service_node = node_combo_->currentData().toString().toStdString();
  autolink::ParameterClient client(node, service_node);
  if (!client.SetParameter(*parsed)) {
    updateStatusText(tr("Failed to set %1.")
                         .arg(QString::fromStdString(parsed->Name())),
                     true);
    return false;
  }

  if (name_item != nullptr) {
    SetRowData(name_item, *parsed);
  }
  updateStatusText(tr("Updated %1.").arg(QString::fromStdString(parsed->Name())));
  return true;
}

void ParametersPanel::onParameterItemChanged(QTableWidgetItem* item) {
  if (suppress_table_signals_ || item == nullptr ||
      item->column() != kColumnValue) {
    return;
  }

  const int row = item->row();
  QTableWidgetItem* name_item = table_->item(row, kColumnName);
  const std::optional<autolink::Parameter> current = DecodeParameterRow(name_item);
  if (!current.has_value()) {
    return;
  }

  const QString previous_value = FormatParameterValue(*current);
  const QString edited_value = item->text();
  if (edited_value == previous_value) {
    return;
  }

  suppress_table_signals_ = true;
  if (!commitParameterEdit(row, edited_value)) {
    item->setText(previous_value);
  } else if (name_item != nullptr) {
    const std::optional<autolink::Parameter> updated = DecodeParameterRow(name_item);
    if (updated.has_value()) {
      item->setText(FormatParameterValue(*updated));
    }
  }
  suppress_table_signals_ = false;
}

}  // namespace parameters_panel
}  // namespace autoviz
