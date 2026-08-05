/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_settings_widget.hpp"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include <algorithm>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/map/map_message_ingest.hpp"

namespace autoviz {
namespace map {
namespace {

QComboBox* MakeEnumCombo(QWidget* parent) {
  auto* combo = new QComboBox(parent);
  combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  return combo;
}

}  // namespace

MapSettingsWidget::MapSettingsWidget(common::VisualizationManager* manager,
                                     QWidget* parent)
    : manager_(manager), config_(DefaultMapPanelConfig()), QWidget(parent) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  auto* general = new QGroupBox(tr("General"), this);
  auto* general_form = new QFormLayout(general);
  title_edit_ = new QLineEdit(general);
  general_form->addRow(tr("Title"), title_edit_);

  base_layer_combo_ = MakeEnumCombo(general);
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kStreet),
                             static_cast<int>(MapBaseLayer::kStreet));
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kSatellite),
                             static_cast<int>(MapBaseLayer::kSatellite));
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kShadedRelief),
                             static_cast<int>(MapBaseLayer::kShadedRelief));
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kCustom),
                             static_cast<int>(MapBaseLayer::kCustom));
  general_form->addRow(tr("Base layer"), base_layer_combo_);

  custom_tile_url_edit_ = new QLineEdit(general);
  custom_tile_url_edit_->setPlaceholderText(tr("https://example.com/{z}/{x}/{y}.png"));
  general_form->addRow(tr("Custom tile URL"), custom_tile_url_edit_);

  follow_channel_combo_ = new QComboBox(general);
  follow_channel_combo_->setEditable(false);
  general_form->addRow(tr("Follow channel"), follow_channel_combo_);

  center_lat_spin_ = new QDoubleSpinBox(general);
  center_lat_spin_->setRange(-85.0, 85.0);
  center_lat_spin_->setDecimals(6);
  center_lon_spin_ = new QDoubleSpinBox(general);
  center_lon_spin_->setRange(-180.0, 180.0);
  center_lon_spin_->setDecimals(6);
  zoom_spin_ = new QDoubleSpinBox(general);
  zoom_spin_->setRange(2.0, 20.0);
  zoom_spin_->setSingleStep(0.5);
  general_form->addRow(tr("Center latitude"), center_lat_spin_);
  general_form->addRow(tr("Center longitude"), center_lon_spin_);
  general_form->addRow(tr("Zoom"), zoom_spin_);
  root->addWidget(general);

  auto* topics_group = new QGroupBox(tr("Topics"), this);
  auto* topics_layout = new QVBoxLayout(topics_group);
  auto* topic_buttons = new QHBoxLayout();
  auto* add_topic = new QPushButton(tr("Add"), topics_group);
  auto* remove_topic = new QPushButton(tr("Remove"), topics_group);
  topic_buttons->addWidget(add_topic);
  topic_buttons->addWidget(remove_topic);
  topic_buttons->addStretch(1);
  topics_layout->addLayout(topic_buttons);
  topic_list_ = new QListWidget(topics_group);
  topics_layout->addWidget(topic_list_);

  topic_editor_ = new QWidget(topics_group);
  auto* topic_form = new QFormLayout(topic_editor_);
  topic_channel_combo_ = new QComboBox(topic_editor_);
  topic_channel_combo_->setEditable(true);
  topic_form->addRow(tr("Channel"), topic_channel_combo_);
  topic_style_combo_ = MakeEnumCombo(topic_editor_);
  topic_style_combo_->addItem(PointStyleLabel(MapPointStyle::kDot),
                              static_cast<int>(MapPointStyle::kDot));
  topic_style_combo_->addItem(PointStyleLabel(MapPointStyle::kArrow),
                              static_cast<int>(MapPointStyle::kArrow));
  topic_style_combo_->addItem(PointStyleLabel(MapPointStyle::kDiamond),
                              static_cast<int>(MapPointStyle::kDiamond));
  topic_style_combo_->addItem(PointStyleLabel(MapPointStyle::kSquare),
                              static_cast<int>(MapPointStyle::kSquare));
  topic_style_combo_->addItem(PointStyleLabel(MapPointStyle::kCross),
                              static_cast<int>(MapPointStyle::kCross));
  topic_form->addRow(tr("Point style"), topic_style_combo_);
  topic_show_heading_check_ = new QCheckBox(tr("Show heading"), topic_editor_);
  topic_show_velocity_check_ = new QCheckBox(tr("Show velocity"), topic_editor_);
  topic_form->addRow(QString(), topic_show_heading_check_);
  topic_form->addRow(QString(), topic_show_velocity_check_);
  topic_point_size_spin_ = new QDoubleSpinBox(topic_editor_);
  topic_point_size_spin_->setRange(2.0, 64.0);
  topic_form->addRow(tr("Point size (px)"), topic_point_size_spin_);
  topic_time_range_combo_ = MakeEnumCombo(topic_editor_);
  topic_time_range_combo_->addItem(TimeRangeLabel(MapTimeRange::kLatest),
                                   static_cast<int>(MapTimeRange::kLatest));
  topic_time_range_combo_->addItem(TimeRangeLabel(MapTimeRange::kLastNSeconds),
                                   static_cast<int>(MapTimeRange::kLastNSeconds));
  topic_time_range_combo_->addItem(TimeRangeLabel(MapTimeRange::kAll),
                                   static_cast<int>(MapTimeRange::kAll));
  topic_form->addRow(tr("Time range"), topic_time_range_combo_);
  topic_time_seconds_spin_ = new QDoubleSpinBox(topic_editor_);
  topic_time_seconds_spin_->setRange(1.0, 3600.0);
  topic_form->addRow(tr("History seconds"), topic_time_seconds_spin_);
  topic_opacity_spin_ = new QDoubleSpinBox(topic_editor_);
  topic_opacity_spin_->setRange(0.0, 1.0);
  topic_opacity_spin_->setSingleStep(0.05);
  topic_form->addRow(tr("Layer opacity"), topic_opacity_spin_);
  topic_color_button_ = new QPushButton(tr("Pick color"), topic_editor_);
  topic_form->addRow(tr("Color"), topic_color_button_);
  topic_enabled_check_ = new QCheckBox(tr("Enabled"), topic_editor_);
  topic_form->addRow(QString(), topic_enabled_check_);
  topics_layout->addWidget(topic_editor_);
  root->addWidget(topics_group);

  auto* overlay_group = new QGroupBox(tr("Overlay layers"), this);
  auto* overlay_layout = new QVBoxLayout(overlay_group);
  auto* overlay_buttons = new QHBoxLayout();
  auto* add_overlay = new QPushButton(tr("Add"), overlay_group);
  auto* remove_overlay = new QPushButton(tr("Remove"), overlay_group);
  overlay_buttons->addWidget(add_overlay);
  overlay_buttons->addWidget(remove_overlay);
  overlay_buttons->addStretch(1);
  overlay_layout->addLayout(overlay_buttons);
  overlay_list_ = new QListWidget(overlay_group);
  overlay_layout->addWidget(overlay_list_);
  overlay_editor_ = new QWidget(overlay_group);
  auto* overlay_form = new QFormLayout(overlay_editor_);
  overlay_name_edit_ = new QLineEdit(overlay_editor_);
  overlay_url_edit_ = new QLineEdit(overlay_editor_);
  overlay_opacity_spin_ = new QDoubleSpinBox(overlay_editor_);
  overlay_opacity_spin_->setRange(0.0, 1.0);
  overlay_opacity_spin_->setSingleStep(0.05);
  overlay_enabled_check_ = new QCheckBox(tr("Enabled"), overlay_editor_);
  overlay_form->addRow(tr("Name"), overlay_name_edit_);
  overlay_form->addRow(tr("Tile URL"), overlay_url_edit_);
  overlay_form->addRow(tr("Opacity"), overlay_opacity_spin_);
  overlay_form->addRow(QString(), overlay_enabled_check_);
  overlay_layout->addWidget(overlay_editor_);
  root->addWidget(overlay_group);
  root->addStretch(1);

  connect(add_topic, &QPushButton::clicked, this, &MapSettingsWidget::onAddTopicLayer);
  connect(remove_topic, &QPushButton::clicked, this, &MapSettingsWidget::onRemoveTopicLayer);
  connect(topic_list_, &QListWidget::currentRowChanged, this,
          &MapSettingsWidget::onTopicSelectionChanged);
  connect(add_overlay, &QPushButton::clicked, this, &MapSettingsWidget::onAddOverlayLayer);
  connect(remove_overlay, &QPushButton::clicked, this,
          &MapSettingsWidget::onRemoveOverlayLayer);
  connect(overlay_list_, &QListWidget::currentRowChanged, this,
          &MapSettingsWidget::onOverlaySelectionChanged);

  const auto wire_change = [this]() { emitConfigChanged(); };
  connect(title_edit_, &QLineEdit::textEdited, this, wire_change);
  connect(base_layer_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(custom_tile_url_edit_, &QLineEdit::textEdited, this, wire_change);
  connect(follow_channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(center_lat_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(center_lon_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(zoom_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(topic_channel_combo_, &QComboBox::currentTextChanged, this, wire_change);
  connect(topic_style_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(topic_show_heading_check_, &QCheckBox::toggled, this, wire_change);
  connect(topic_show_velocity_check_, &QCheckBox::toggled, this, wire_change);
  connect(topic_point_size_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(topic_time_range_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(topic_time_seconds_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(topic_opacity_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(topic_enabled_check_, &QCheckBox::toggled, this, wire_change);
  connect(overlay_name_edit_, &QLineEdit::textEdited, this, wire_change);
  connect(overlay_url_edit_, &QLineEdit::textEdited, this, wire_change);
  connect(overlay_opacity_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(overlay_enabled_check_, &QCheckBox::toggled, this, wire_change);
  connect(topic_color_button_, &QPushButton::clicked, this, [this]() {
    if (selected_topic_index_ < 0 ||
        selected_topic_index_ >= config_.topic_layers.size()) {
      return;
    }
    const QColor picked = QColorDialog::getColor(
        config_.topic_layers.at(selected_topic_index_).color, this, tr("Point color"));
    if (!picked.isValid()) {
      return;
    }
    config_.topic_layers[selected_topic_index_].color = picked;
    topic_color_button_->setStyleSheet(
        QStringLiteral("background:%1;").arg(picked.name()));
    emitConfigChanged();
  });

  setConfig(config_);
  refreshChannels();
}

MapPanelConfig MapSettingsWidget::config() {
  MapPanelConfig result = config_;
  result.title = title_edit_->text().trimmed();
  result.base_layer =
      static_cast<MapBaseLayer>(base_layer_combo_->currentData().toInt());
  result.custom_tile_url = custom_tile_url_edit_->text().trimmed();
  result.follow_channel = follow_channel_combo_->currentData().toString();
  result.center_latitude = center_lat_spin_->value();
  result.center_longitude = center_lon_spin_->value();
  result.zoom = zoom_spin_->value();
  saveCurrentTopicEditor();
  saveCurrentOverlayEditor();
  return result;
}

void MapSettingsWidget::setConfig(const MapPanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  base_layer_combo_->setCurrentIndex(
      base_layer_combo_->findData(static_cast<int>(config_.base_layer)));
  custom_tile_url_edit_->setText(config_.custom_tile_url);
  center_lat_spin_->setValue(config_.center_latitude);
  center_lon_spin_->setValue(config_.center_longitude);
  zoom_spin_->setValue(config_.zoom);
  rebuildTopicList();
  rebuildOverlayList();
  refreshChannels();
  const int follow_index =
      follow_channel_combo_->findData(config_.follow_channel);
  follow_channel_combo_->setCurrentIndex(follow_index >= 0 ? follow_index : 0);
}

void MapSettingsWidget::refreshChannels() {
  if (manager_ == nullptr) {
    return;
  }
  const QString previous_follow = follow_channel_combo_->currentData().toString();
  const QString previous_topic = topic_channel_combo_->currentText();
  follow_channel_combo_->clear();
  follow_channel_combo_->addItem(tr("(None)"), QString());
  topic_channel_combo_->clear();
  for (const integration::ChannelInfo& info : manager_->channels()) {
    const QString channel = QString::fromStdString(info.channel_name);
    if (channel.isEmpty()) {
      continue;
    }
    if (MapMessageIngest::SupportsMessageType(
            QString::fromStdString(info.message_type))) {
      follow_channel_combo_->addItem(channel, channel);
      topic_channel_combo_->addItem(channel);
    }
  }
  const int follow_index = follow_channel_combo_->findData(previous_follow);
  follow_channel_combo_->setCurrentIndex(follow_index >= 0 ? follow_index : 0);
  const int topic_index = topic_channel_combo_->findText(previous_topic);
  if (topic_index >= 0) {
    topic_channel_combo_->setCurrentIndex(topic_index);
  }
}

void MapSettingsWidget::rebuildTopicList() {
  topic_list_->clear();
  for (const MapTopicLayerConfig& layer : config_.topic_layers) {
    topic_list_->addItem(layer.channel.isEmpty() ? tr("(Unconfigured)") : layer.channel);
  }
  if (config_.topic_layers.isEmpty()) {
    selected_topic_index_ = -1;
    topic_editor_->setEnabled(false);
    return;
  }
  selected_topic_index_ = std::clamp(
      selected_topic_index_, 0,
      static_cast<int>(config_.topic_layers.size()) - 1);
  topic_list_->setCurrentRow(selected_topic_index_);
  loadTopicEditor(config_.topic_layers.at(selected_topic_index_));
  topic_editor_->setEnabled(true);
}

void MapSettingsWidget::rebuildOverlayList() {
  overlay_list_->clear();
  for (const MapOverlayLayerConfig& layer : config_.overlay_layers) {
    overlay_list_->addItem(layer.name.isEmpty() ? tr("Overlay") : layer.name);
  }
  if (config_.overlay_layers.isEmpty()) {
    selected_overlay_index_ = -1;
    overlay_editor_->setEnabled(false);
    return;
  }
  selected_overlay_index_ = std::clamp(
      selected_overlay_index_, 0,
      static_cast<int>(config_.overlay_layers.size()) - 1);
  overlay_list_->setCurrentRow(selected_overlay_index_);
  loadOverlayEditor(config_.overlay_layers.at(selected_overlay_index_));
  overlay_editor_->setEnabled(true);
}

void MapSettingsWidget::loadTopicEditor(const MapTopicLayerConfig& layer) {
  const int channel_index = topic_channel_combo_->findText(layer.channel);
  if (channel_index >= 0) {
    topic_channel_combo_->setCurrentIndex(channel_index);
  } else {
    topic_channel_combo_->setEditText(layer.channel);
  }
  topic_style_combo_->setCurrentIndex(
      topic_style_combo_->findData(static_cast<int>(layer.point_style)));
  topic_show_heading_check_->setChecked(layer.show_heading);
  topic_show_velocity_check_->setChecked(layer.show_velocity);
  topic_point_size_spin_->setValue(layer.point_size);
  topic_time_range_combo_->setCurrentIndex(
      topic_time_range_combo_->findData(static_cast<int>(layer.time_range)));
  topic_time_seconds_spin_->setValue(layer.time_range_seconds);
  topic_opacity_spin_->setValue(layer.layer_opacity);
  topic_enabled_check_->setChecked(layer.enabled);
  topic_color_button_->setStyleSheet(
      layer.color.isValid()
          ? QStringLiteral("background:%1;").arg(layer.color.name())
          : QString());
}

void MapSettingsWidget::loadOverlayEditor(const MapOverlayLayerConfig& layer) {
  overlay_name_edit_->setText(layer.name);
  overlay_url_edit_->setText(layer.tile_url_template);
  overlay_opacity_spin_->setValue(layer.opacity);
  overlay_enabled_check_->setChecked(layer.enabled);
}

MapTopicLayerConfig MapSettingsWidget::readTopicEditor() const {
  MapTopicLayerConfig layer;
  layer.channel = topic_channel_combo_->currentText().trimmed();
  layer.point_style =
      static_cast<MapPointStyle>(topic_style_combo_->currentData().toInt());
  layer.show_heading = topic_show_heading_check_->isChecked();
  layer.show_velocity = topic_show_velocity_check_->isChecked();
  layer.point_size = topic_point_size_spin_->value();
  layer.time_range =
      static_cast<MapTimeRange>(topic_time_range_combo_->currentData().toInt());
  layer.time_range_seconds = topic_time_seconds_spin_->value();
  layer.layer_opacity = topic_opacity_spin_->value();
  layer.enabled = topic_enabled_check_->isChecked();
  if (selected_topic_index_ >= 0 && selected_topic_index_ < config_.topic_layers.size()) {
    layer.color = config_.topic_layers.at(selected_topic_index_).color;
  }
  return layer;
}

MapOverlayLayerConfig MapSettingsWidget::readOverlayEditor() const {
  MapOverlayLayerConfig layer;
  layer.name = overlay_name_edit_->text().trimmed();
  layer.tile_url_template = overlay_url_edit_->text().trimmed();
  layer.opacity = overlay_opacity_spin_->value();
  layer.enabled = overlay_enabled_check_->isChecked();
  return layer;
}

void MapSettingsWidget::saveCurrentTopicEditor() {
  if (selected_topic_index_ < 0 || selected_topic_index_ >= config_.topic_layers.size()) {
    return;
  }
  config_.topic_layers[selected_topic_index_] = readTopicEditor();
  if (topic_list_->currentRow() == selected_topic_index_) {
    topic_list_->item(selected_topic_index_)->setText(
        config_.topic_layers.at(selected_topic_index_).channel.isEmpty()
            ? tr("(Unconfigured)")
            : config_.topic_layers.at(selected_topic_index_).channel);
  }
}

void MapSettingsWidget::saveCurrentOverlayEditor() {
  if (selected_overlay_index_ < 0 ||
      selected_overlay_index_ >= config_.overlay_layers.size()) {
    return;
  }
  config_.overlay_layers[selected_overlay_index_] = readOverlayEditor();
  if (overlay_list_->currentRow() == selected_overlay_index_) {
    overlay_list_->item(selected_overlay_index_)->setText(
        config_.overlay_layers.at(selected_overlay_index_).name.isEmpty()
            ? tr("Overlay")
            : config_.overlay_layers.at(selected_overlay_index_).name);
  }
}

QColor MapSettingsWidget::defaultColorForIndex(int index) const {
  static const QColor palette[] = {QColor(255, 90, 60),  QColor(0, 170, 255),
                                   QColor(120, 220, 90), QColor(255, 190, 40),
                                   QColor(180, 120, 255), QColor(255, 120, 180)};
  return palette[index % 6];
}

void MapSettingsWidget::onAddTopicLayer() {
  saveCurrentTopicEditor();
  MapTopicLayerConfig layer;
  layer.color = defaultColorForIndex(config_.topic_layers.size());
  config_.topic_layers.push_back(layer);
  selected_topic_index_ = config_.topic_layers.size() - 1;
  rebuildTopicList();
  emitConfigChanged();
}

void MapSettingsWidget::onRemoveTopicLayer() {
  if (selected_topic_index_ < 0 || selected_topic_index_ >= config_.topic_layers.size()) {
    return;
  }
  config_.topic_layers.removeAt(selected_topic_index_);
  selected_topic_index_ =
      std::min(selected_topic_index_, static_cast<int>(config_.topic_layers.size()) - 1);
  rebuildTopicList();
  emitConfigChanged();
}

void MapSettingsWidget::onTopicSelectionChanged() {
  saveCurrentTopicEditor();
  selected_topic_index_ = topic_list_->currentRow();
  if (selected_topic_index_ < 0 || selected_topic_index_ >= config_.topic_layers.size()) {
    topic_editor_->setEnabled(false);
    return;
  }
  loadTopicEditor(config_.topic_layers.at(selected_topic_index_));
  topic_editor_->setEnabled(true);
}

void MapSettingsWidget::onAddOverlayLayer() {
  saveCurrentOverlayEditor();
  MapOverlayLayerConfig layer;
  layer.name = tr("Overlay %1").arg(config_.overlay_layers.size() + 1);
  layer.opacity = 0.5;
  config_.overlay_layers.push_back(layer);
  selected_overlay_index_ = config_.overlay_layers.size() - 1;
  rebuildOverlayList();
  emitConfigChanged();
}

void MapSettingsWidget::onRemoveOverlayLayer() {
  if (selected_overlay_index_ < 0 ||
      selected_overlay_index_ >= config_.overlay_layers.size()) {
    return;
  }
  config_.overlay_layers.removeAt(selected_overlay_index_);
  selected_overlay_index_ =
      std::min(selected_overlay_index_,
               static_cast<int>(config_.overlay_layers.size()) - 1);
  rebuildOverlayList();
  emitConfigChanged();
}

void MapSettingsWidget::onOverlaySelectionChanged() {
  saveCurrentOverlayEditor();
  selected_overlay_index_ = overlay_list_->currentRow();
  if (selected_overlay_index_ < 0 ||
      selected_overlay_index_ >= config_.overlay_layers.size()) {
    overlay_editor_->setEnabled(false);
    return;
  }
  loadOverlayEditor(config_.overlay_layers.at(selected_overlay_index_));
  overlay_editor_->setEnabled(true);
}

void MapSettingsWidget::emitConfigChanged() {
  config_ = config();
  emit configChanged();
}

}  // namespace map
}  // namespace autoviz
