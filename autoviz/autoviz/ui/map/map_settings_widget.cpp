/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_settings_widget.hpp"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

#include <algorithm>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/map/map_message_ingest.hpp"
#include "autoviz/ui/map/map_plan.hpp"
#include "autoviz/ui/map/map_types.hpp"
#include "autoviz/ui/theme/panel.hpp"

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
  ApplyCompactSettingsShell(this);
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                           PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  root->setSpacing(PanelSettingsLayout::kOuterSpacing);
  root->setAlignment(Qt::AlignTop);

  attribute_group_ = new QGroupBox(tr("Attributes"), this);
  StyleSettingsGroupBox(attribute_group_);
  auto* attribute_layout = new QVBoxLayout(attribute_group_);
  ApplyCompactVBox(attribute_layout);
  attribute_hint_ = new QLabel(
      tr("Select a feature or plan vertex on the map."), attribute_group_);
  attribute_hint_->setWordWrap(true);
  attribute_hint_->setStyleSheet(PropertyInspectorHintStyle());
  attribute_layout->addWidget(attribute_hint_);
  attribute_body_ = new QWidget(attribute_group_);
  auto* attribute_form = new QFormLayout(attribute_body_);
  ApplyCompactForm(attribute_form);
  attribute_kind_label_ = new QLabel(attribute_body_);
  attribute_layer_label_ = new QLabel(attribute_body_);
  attribute_kind_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  attribute_layer_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  attribute_lat_spin_ = new QDoubleSpinBox(attribute_body_);
  attribute_lat_spin_->setRange(-85.0, 85.0);
  attribute_lat_spin_->setDecimals(7);
  attribute_lon_spin_ = new QDoubleSpinBox(attribute_body_);
  attribute_lon_spin_->setRange(-180.0, 180.0);
  attribute_lon_spin_->setDecimals(7);
  attribute_form->addRow(tr("Type"), attribute_kind_label_);
  attribute_form->addRow(tr("Layer"), attribute_layer_label_);
  attribute_form->addRow(tr("Latitude"), attribute_lat_spin_);
  attribute_form->addRow(tr("Longitude"), attribute_lon_spin_);
  attribute_table_ = new QTableWidget(0, 2, attribute_body_);
  attribute_table_->setHorizontalHeaderLabels({tr("Property"), tr("Value")});
  attribute_table_->verticalHeader()->hide();
  attribute_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  attribute_table_->setSelectionMode(QAbstractItemView::NoSelection);
  attribute_table_->setFocusPolicy(Qt::NoFocus);
  attribute_table_->setShowGrid(false);
  attribute_table_->setWordWrap(true);
  attribute_table_->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::ResizeToContents);
  attribute_table_->horizontalHeader()->setStretchLastSection(true);
  attribute_table_->setMaximumHeight(220);
  attribute_form->addRow(attribute_table_);
  attribute_remove_button_ =
      MakeDestructiveFlatActionButton(tr("Remove vertex"), attribute_body_);
  attribute_form->addRow(QString(), attribute_remove_button_);
  attribute_body_->hide();
  attribute_layout->addWidget(attribute_body_);
  root->addWidget(attribute_group_);

  auto* general = new QGroupBox(tr("General"), this);
  StyleSettingsGroupBox(general);
  auto* general_form = new QFormLayout(general);
  ApplyCompactForm(general_form);
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
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kEsriStreet),
                             static_cast<int>(MapBaseLayer::kEsriStreet));
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kEsriTerrain),
                             static_cast<int>(MapBaseLayer::kEsriTerrain));
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kCartoVoyager),
                             static_cast<int>(MapBaseLayer::kCartoVoyager));
  base_layer_combo_->addItem(BaseLayerLabel(MapBaseLayer::kJapanStandard),
                             static_cast<int>(MapBaseLayer::kJapanStandard));
  general_form->addRow(tr("Base layer"), base_layer_combo_);

  custom_tile_url_edit_ = new QLineEdit(general);
  custom_tile_url_edit_->setPlaceholderText(tr("https://example.com/{z}/{x}/{y}.png"));
  general_form->addRow(tr("Custom tile URL"), custom_tile_url_edit_);

  follow_channel_combo_ = new QComboBox(general);
  follow_channel_combo_->setEditable(false);
  general_form->addRow(tr("Follow channel"), follow_channel_combo_);

  gcs_channel_combo_ = new QComboBox(general);
  gcs_channel_combo_->setEditable(false);
  general_form->addRow(tr("GCS channel"), gcs_channel_combo_);

  distance_unit_combo_ = MakeEnumCombo(general);
  distance_unit_combo_->addItem(DistanceUnitLabel(MapDistanceUnit::kMeters),
                                static_cast<int>(MapDistanceUnit::kMeters));
  distance_unit_combo_->addItem(DistanceUnitLabel(MapDistanceUnit::kFeet),
                                static_cast<int>(MapDistanceUnit::kFeet));
  general_form->addRow(tr("Distance units"), distance_unit_combo_);

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

  auto* topics_group = new QGroupBox(tr("Channels"), this);
  StyleSettingsGroupBox(topics_group);
  auto* topics_layout = new QVBoxLayout(topics_group);
  ApplyCompactVBox(topics_layout);
  auto* topic_buttons = new QHBoxLayout();
  auto* add_topic = MakeFlatActionButton(tr("Add"), topics_group);
  auto* remove_topic = MakeDestructiveFlatActionButton(tr("Remove"), topics_group);
  topic_buttons->addWidget(add_topic);
  topic_buttons->addWidget(remove_topic);
  topic_buttons->addStretch(1);
  topics_layout->addLayout(topic_buttons);
  topic_list_ = new QListWidget(topics_group);
  topics_layout->addWidget(topic_list_);

  topic_editor_ = new QWidget(topics_group);
  auto* topic_form = new QFormLayout(topic_editor_);
  ApplyCompactForm(topic_form);
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
  topic_color_button_ = MakeFlatActionButton(tr("Pick color"), topic_editor_);
  topic_form->addRow(tr("Color"), topic_color_button_);
  topic_enabled_check_ = new QCheckBox(tr("Enabled"), topic_editor_);
  topic_form->addRow(QString(), topic_enabled_check_);
  topics_layout->addWidget(topic_editor_);
  root->addWidget(topics_group);

  auto* overlay_group = new QGroupBox(tr("Overlay layers"), this);
  StyleSettingsGroupBox(overlay_group);
  auto* overlay_layout = new QVBoxLayout(overlay_group);
  ApplyCompactVBox(overlay_layout);
  auto* overlay_buttons = new QHBoxLayout();
  auto* add_overlay = MakeFlatActionButton(tr("Add"), overlay_group);
  auto* remove_overlay = MakeDestructiveFlatActionButton(tr("Remove"), overlay_group);
  overlay_buttons->addWidget(add_overlay);
  overlay_buttons->addWidget(remove_overlay);
  overlay_buttons->addStretch(1);
  overlay_layout->addLayout(overlay_buttons);
  overlay_list_ = new QListWidget(overlay_group);
  overlay_layout->addWidget(overlay_list_);
  overlay_editor_ = new QWidget(overlay_group);
  auto* overlay_form = new QFormLayout(overlay_editor_);
  ApplyCompactForm(overlay_form);
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

  auto* geojson_group = new QGroupBox(tr("GeoJSON"), this);
  StyleSettingsGroupBox(geojson_group);
  auto* geojson_layout = new QVBoxLayout(geojson_group);
  ApplyCompactVBox(geojson_layout);
  auto* geojson_buttons = new QHBoxLayout();
  auto* remove_geojson = MakeDestructiveFlatActionButton(tr("Remove"), geojson_group);
  geojson_buttons->addWidget(remove_geojson);
  geojson_buttons->addStretch(1);
  geojson_layout->addLayout(geojson_buttons);
  geojson_list_ = new QListWidget(geojson_group);
  geojson_layout->addWidget(geojson_list_);
  root->addWidget(geojson_group);

  auto* plan_group = new QGroupBox(tr("Plan"), this);
  StyleSettingsGroupBox(plan_group);
  auto* plan_form = new QFormLayout(plan_group);
  ApplyCompactForm(plan_form);
  edit_tool_combo_ = MakeEnumCombo(plan_group);
  edit_tool_combo_->addItem(EditToolLabel(MapEditTool::kPan),
                            static_cast<int>(MapEditTool::kPan));
  edit_tool_combo_->addItem(EditToolLabel(MapEditTool::kWaypoint),
                            static_cast<int>(MapEditTool::kWaypoint));
  edit_tool_combo_->addItem(EditToolLabel(MapEditTool::kGeofence),
                            static_cast<int>(MapEditTool::kGeofence));
  edit_tool_combo_->addItem(EditToolLabel(MapEditTool::kRally),
                            static_cast<int>(MapEditTool::kRally));
  edit_tool_combo_->setToolTip(tr("1 Pan · 2 Waypoint · 3 Geofence · 4 Rally · M Measure · Esc Cancel · Del Undo · F Fit"));
  plan_form->addRow(tr("Click tool"), edit_tool_combo_);
  survey_spacing_spin_ = new QDoubleSpinBox(plan_group);
  survey_spacing_spin_->setRange(5.0, 500.0);
  survey_spacing_spin_->setSuffix(tr(" m"));
  plan_form->addRow(tr("Survey spacing"), survey_spacing_spin_);
  corridor_width_spin_ = new QDoubleSpinBox(plan_group);
  corridor_width_spin_->setRange(5.0, 500.0);
  corridor_width_spin_->setSuffix(tr(" m"));
  plan_form->addRow(tr("Corridor width"), corridor_width_spin_);
  structure_radius_spin_ = new QDoubleSpinBox(plan_group);
  structure_radius_spin_->setRange(5.0, 2000.0);
  structure_radius_spin_->setSuffix(tr(" m"));
  plan_form->addRow(tr("Structure radius"), structure_radius_spin_);
  auto* plan_buttons = new QHBoxLayout();
  auto* survey_button = MakePrimaryActionButton(tr("Survey"), plan_group);
  auto* corridor_button = MakeFlatActionButton(tr("Corridor"), plan_group);
  auto* structure_button = MakeFlatActionButton(tr("Structure"), plan_group);
  plan_buttons->addWidget(survey_button);
  plan_buttons->addWidget(corridor_button);
  plan_buttons->addWidget(structure_button);
  plan_form->addRow(QString(), plan_buttons);
  auto* file_buttons = new QHBoxLayout();
  auto* export_plan = MakeFlatActionButton(tr("Export"), plan_group);
  auto* import_plan = MakeFlatActionButton(tr("Import"), plan_group);
  file_buttons->addWidget(export_plan);
  file_buttons->addWidget(import_plan);
  plan_form->addRow(tr("Plan file"), file_buttons);
  root->addWidget(plan_group);

  auto* offline_group = new QGroupBox(tr("Offline tiles"), this);
  StyleSettingsGroupBox(offline_group);
  auto* offline_form = new QFormLayout(offline_group);
  ApplyCompactForm(offline_form);
  offline_min_zoom_spin_ = new QSpinBox(offline_group);
  offline_min_zoom_spin_->setRange(2, 18);
  offline_max_zoom_spin_ = new QSpinBox(offline_group);
  offline_max_zoom_spin_->setRange(2, 18);
  offline_min_zoom_spin_->setValue(14);
  offline_max_zoom_spin_->setValue(16);
  offline_form->addRow(tr("Min zoom"), offline_min_zoom_spin_);
  offline_form->addRow(tr("Max zoom"), offline_max_zoom_spin_);
  auto* download_button = MakePrimaryActionButton(tr("Download view"), offline_group);
  offline_form->addRow(QString(), download_button);
  root->addWidget(offline_group);
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
  connect(remove_geojson, &QPushButton::clicked, this, [this]() {
    const int row = geojson_list_->currentRow();
    if (row < 0) {
      return;
    }
    delete geojson_list_->takeItem(row);
    emitConfigChanged();
  });
  connect(geojson_list_, &QListWidget::itemChanged, this, [this]() { emitConfigChanged(); });
  connect(survey_button, &QPushButton::clicked, this, [this]() {
    if (config_.geofence.size() < 3) {
      return;
    }
    config_.waypoints = SurveyLawnmower(config_.geofence, survey_spacing_spin_->value());
    emitConfigChanged();
  });
  connect(corridor_button, &QPushButton::clicked, this, [this]() {
    if (config_.waypoints.size() < 2) {
      return;
    }
    config_.waypoints = CorridorScan(config_.waypoints, corridor_width_spin_->value());
    emitConfigChanged();
  });
  connect(structure_button, &QPushButton::clicked, this, [this]() {
    config_.waypoints = StructureScan(config_.center_latitude, config_.center_longitude,
                                      structure_radius_spin_->value(), 12);
    emitConfigChanged();
  });
  connect(export_plan, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Export plan"), QString(), tr("GeoJSON (*.geojson *.json)"));
    if (path.isEmpty()) {
      return;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
      return;
    }
    file.write(ExportPlanGeoJson(config_.waypoints, config_.geofence, config_.rally_points));
  });
  connect(import_plan, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Import plan"), QString(), tr("GeoJSON (*.geojson *.json)"));
    if (path.isEmpty()) {
      return;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
      return;
    }
    QString error;
    if (!ImportPlanGeoJson(file.readAll(), &config_.waypoints, &config_.geofence,
                           &config_.rally_points, &error)) {
      return;
    }
    emitConfigChanged();
  });
  connect(download_button, &QPushButton::clicked, this, [this]() {
    emit offlineDownloadRequested(offline_min_zoom_spin_->value(),
                                  offline_max_zoom_spin_->value());
  });
  const auto edit_plan_vertex = [this]() {
    if (selection_.plan_kind < 0 || attribute_lat_spin_->isReadOnly()) {
      return;
    }
    emit planVertexEdited(selection_.plan_kind, selection_.plan_index,
                          attribute_lat_spin_->value(), attribute_lon_spin_->value());
  };
  connect(attribute_lat_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          edit_plan_vertex);
  connect(attribute_lon_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          edit_plan_vertex);
  connect(attribute_remove_button_, &QPushButton::clicked, this, [this]() {
    if (selection_.plan_kind < 0) {
      return;
    }
    emit planVertexRemoved(selection_.plan_kind, selection_.plan_index);
  });

  const auto wire_change = [this]() { emitConfigChanged(); };
  connect(title_edit_, &QLineEdit::textEdited, this, wire_change);
  connect(base_layer_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(custom_tile_url_edit_, &QLineEdit::textEdited, this, wire_change);
  connect(follow_channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(gcs_channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(distance_unit_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(edit_tool_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          wire_change);
  connect(survey_spacing_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(corridor_width_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          wire_change);
  connect(structure_radius_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
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
    UpdateColorButton(topic_color_button_, picked);
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
  result.gcs_channel = gcs_channel_combo_->currentData().toString();
  result.distance_unit = static_cast<MapDistanceUnit>(
      distance_unit_combo_->currentData().toInt());
  result.edit_tool =
      static_cast<MapEditTool>(edit_tool_combo_->currentData().toInt());
  result.survey_spacing_m = survey_spacing_spin_->value();
  result.corridor_width_m = corridor_width_spin_->value();
  result.structure_radius_m = structure_radius_spin_->value();
  result.center_latitude = center_lat_spin_->value();
  result.center_longitude = center_lon_spin_->value();
  result.zoom = zoom_spin_->value();
  result.geojson_sources.clear();
  for (int i = 0; i < geojson_list_->count(); ++i) {
    const QListWidgetItem* item = geojson_list_->item(i);
    MapGeoJsonSource source;
    source.path = item->data(Qt::UserRole).toString();
    source.visible = item->checkState() == Qt::Checked;
    if (!source.path.isEmpty()) {
      result.geojson_sources.push_back(source);
    }
  }
  saveCurrentTopicEditor();
  saveCurrentOverlayEditor();
  return result;
}

void MapSettingsWidget::setSelection(const MapSelectionInfo& info) {
  bool same_rows = selection_.attributes.size() == info.attributes.size();
  if (same_rows) {
    for (int i = 0; i < info.attributes.size(); ++i) {
      if (selection_.attributes.at(i).key != info.attributes.at(i).key ||
          selection_.attributes.at(i).value != info.attributes.at(i).value) {
        same_rows = false;
        break;
      }
    }
  }
  selection_ = info;
  const bool show = info.valid;
  attribute_body_->setVisible(show);
  attribute_hint_->setVisible(!show);
  attribute_group_->setTitle(show && !info.title.isEmpty() ? info.title
                                                           : tr("Attributes"));
  if (!show) {
    attribute_table_->setRowCount(0);
    return;
  }
  attribute_kind_label_->setText(info.kind);
  attribute_layer_label_->setText(info.layer);
  const bool editable = info.plan_kind >= 0;
  {
    const QSignalBlocker block_lat(attribute_lat_spin_);
    const QSignalBlocker block_lon(attribute_lon_spin_);
    attribute_lat_spin_->setValue(info.latitude);
    attribute_lon_spin_->setValue(info.longitude);
  }
  attribute_lat_spin_->setReadOnly(!editable);
  attribute_lon_spin_->setReadOnly(!editable);
  attribute_lat_spin_->setButtonSymbols(editable ? QAbstractSpinBox::UpDownArrows
                                                : QAbstractSpinBox::NoButtons);
  attribute_lon_spin_->setButtonSymbols(editable ? QAbstractSpinBox::UpDownArrows
                                                : QAbstractSpinBox::NoButtons);
  attribute_remove_button_->setVisible(editable);
  attribute_table_->setVisible(!info.attributes.isEmpty());
  if (same_rows) {
    return;
  }
  attribute_table_->setRowCount(info.attributes.size());
  for (int row = 0; row < info.attributes.size(); ++row) {
    auto* key = new QTableWidgetItem(info.attributes.at(row).key);
    auto* value = new QTableWidgetItem(info.attributes.at(row).value);
    key->setFlags(Qt::ItemIsEnabled);
    value->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    attribute_table_->setItem(row, 0, key);
    attribute_table_->setItem(row, 1, value);
  }
  attribute_table_->resizeRowsToContents();
}

void MapSettingsWidget::setConfig(const MapPanelConfig& config) {
  const QSignalBlocker block_base(base_layer_combo_);
  const QSignalBlocker block_follow(follow_channel_combo_);
  const QSignalBlocker block_gcs(gcs_channel_combo_);
  const QSignalBlocker block_units(distance_unit_combo_);
  const QSignalBlocker block_tool(edit_tool_combo_);
  const QSignalBlocker block_spacing(survey_spacing_spin_);
  const QSignalBlocker block_corridor(corridor_width_spin_);
  const QSignalBlocker block_radius(structure_radius_spin_);
  const QSignalBlocker block_lat(center_lat_spin_);
  const QSignalBlocker block_lon(center_lon_spin_);
  const QSignalBlocker block_zoom(zoom_spin_);
  const QSignalBlocker block_topics(topic_list_);
  const QSignalBlocker block_overlays(overlay_list_);
  const QSignalBlocker block_topic(topic_channel_combo_);
  config_ = config;
  rebuildGeoJsonList();
  title_edit_->setText(config.title);
  base_layer_combo_->setCurrentIndex(
      base_layer_combo_->findData(static_cast<int>(config.base_layer)));
  custom_tile_url_edit_->setText(config.custom_tile_url);
  center_lat_spin_->setValue(config.center_latitude);
  center_lon_spin_->setValue(config.center_longitude);
  zoom_spin_->setValue(config.zoom);
  rebuildTopicList();
  rebuildOverlayList();
  refreshChannels();
  const int follow_index = follow_channel_combo_->findData(config.follow_channel);
  follow_channel_combo_->setCurrentIndex(follow_index >= 0 ? follow_index : 0);
  const int gcs_index = gcs_channel_combo_->findData(config.gcs_channel);
  gcs_channel_combo_->setCurrentIndex(gcs_index >= 0 ? gcs_index : 0);
  distance_unit_combo_->setCurrentIndex(
      distance_unit_combo_->findData(static_cast<int>(config.distance_unit)));
  edit_tool_combo_->setCurrentIndex(
      edit_tool_combo_->findData(static_cast<int>(config.edit_tool)));
  survey_spacing_spin_->setValue(config.survey_spacing_m);
  corridor_width_spin_->setValue(config.corridor_width_m);
  structure_radius_spin_->setValue(config.structure_radius_m);
}

void MapSettingsWidget::refreshChannels() {
  if (manager_ == nullptr) {
    return;
  }
  const QString previous_follow = follow_channel_combo_->currentData().toString();
  const QString previous_gcs = gcs_channel_combo_->currentData().toString();
  const QString previous_topic = topic_channel_combo_->currentText();
  follow_channel_combo_->clear();
  gcs_channel_combo_->clear();
  follow_channel_combo_->addItem(tr("(None)"), QString());
  gcs_channel_combo_->addItem(tr("(None)"), QString());
  topic_channel_combo_->clear();
  for (const integration::ChannelInfo& info : manager_->channels()) {
    const QString channel = QString::fromStdString(info.channel_name);
    if (channel.isEmpty()) {
      continue;
    }
    if (MapMessageIngest::SupportsMessageType(
            QString::fromStdString(info.message_type))) {
      follow_channel_combo_->addItem(channel, channel);
      gcs_channel_combo_->addItem(channel, channel);
      topic_channel_combo_->addItem(channel);
    }
  }
  const int follow_index = follow_channel_combo_->findData(previous_follow);
  follow_channel_combo_->setCurrentIndex(follow_index >= 0 ? follow_index : 0);
  const int gcs_index = gcs_channel_combo_->findData(previous_gcs);
  gcs_channel_combo_->setCurrentIndex(gcs_index >= 0 ? gcs_index : 0);
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

void MapSettingsWidget::rebuildGeoJsonList() {
  if (geojson_list_ == nullptr) {
    return;
  }
  geojson_list_->blockSignals(true);
  geojson_list_->clear();
  for (const MapGeoJsonSource& source : config_.geojson_sources) {
    const QString name = QFileInfo(source.path).fileName();
    auto* item = new QListWidgetItem(name.isEmpty() ? source.path : name);
    item->setToolTip(source.path);
    item->setData(Qt::UserRole, source.path);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(source.visible ? Qt::Checked : Qt::Unchecked);
    geojson_list_->addItem(item);
  }
  geojson_list_->blockSignals(false);
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
  UpdateColorButton(topic_color_button_, layer.color);
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
