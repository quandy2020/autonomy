/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_settings_widget.hpp"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QToolButton>
#include <QVBoxLayout>

#include <string>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace image {
namespace {

}  // namespace

ImageSettingsWidget::ImageSettingsWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), QWidget(parent) {
  ApplyCompactSettingsShell(this);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  outer->setSpacing(PanelSettingsLayout::kOuterSpacing);
  outer->setAlignment(Qt::AlignTop);

  auto* title_form = new QFormLayout();
  ApplyCompactForm(title_form);
  title_edit_ = new QLineEdit(config_.title, this);
  title_form->addRow(tr("Title"), title_edit_);
  outer->addLayout(title_form);
  connect(title_edit_, &QLineEdit::textChanged, this,
          &ImageSettingsWidget::emitConfigChanged);

  auto* general_body = new QWidget(this);
  auto* general_form = new QFormLayout(general_body);
  ApplyCompactForm(general_form);
  topic_combo_ = new QComboBox(general_body);
  topic_combo_->setEditable(true);
  general_form->addRow(tr("Topic"), topic_combo_);
  calibration_combo_ = new QComboBox(general_body);
  calibration_combo_->setEditable(true);
  calibration_combo_->addItem(QString(), QString());
  general_form->addRow(tr("Calibration"), calibration_combo_);
  strict_sync_check_ = new QCheckBox(tr("Strict time sync"), general_body);
  general_form->addRow(QString(), strict_sync_check_);
  undistort_check_ = new QCheckBox(tr("Undistort image"), general_body);
  general_form->addRow(QString(), undistort_check_);
  flip_h_check_ = new QCheckBox(tr("Flip horizontal"), general_body);
  flip_v_check_ = new QCheckBox(tr("Flip vertical"), general_body);
  general_form->addRow(QString(), flip_h_check_);
  general_form->addRow(QString(), flip_v_check_);
  rotation_combo_ = new QComboBox(general_body);
  rotation_combo_->addItem(tr("0°"), static_cast<int>(ImageRotation::k0));
  rotation_combo_->addItem(tr("90°"), static_cast<int>(ImageRotation::k90));
  rotation_combo_->addItem(tr("180°"), static_cast<int>(ImageRotation::k180));
  rotation_combo_->addItem(tr("270°"), static_cast<int>(ImageRotation::k270));
  general_form->addRow(tr("Rotation"), rotation_combo_);
  color_mode_combo_ = new QComboBox(general_body);
  color_mode_combo_->addItem(tr("Off"), static_cast<int>(ImageColorMode::kOff));
  color_mode_combo_->addItem(tr("Turbo"), static_cast<int>(ImageColorMode::kTurbo));
  color_mode_combo_->addItem(tr("Rainbow"), static_cast<int>(ImageColorMode::kRainbow));
  color_mode_combo_->addItem(tr("Grayscale"),
                             static_cast<int>(ImageColorMode::kGrayscale));
  general_form->addRow(tr("Color mode"), color_mode_combo_);
  color_min_spin_ = new QDoubleSpinBox(general_body);
  color_min_spin_->setRange(-1e9, 1e9);
  color_min_spin_->setValue(config_.color_min);
  color_max_spin_ = new QDoubleSpinBox(general_body);
  color_max_spin_->setRange(-1e9, 1e9);
  color_max_spin_->setValue(config_.color_max);
  general_form->addRow(tr("Value min"), color_min_spin_);
  general_form->addRow(tr("Value max"), color_max_spin_);
  outer->addWidget(MakeCollapsibleSection(this, tr("General"), general_body, true));

  auto* overlay_body = new QWidget(this);
  auto* overlay_layout = new QVBoxLayout(overlay_body);
  overlay_list_layout_ = new QVBoxLayout();
  overlay_layout->addLayout(overlay_list_layout_);
  add_overlay_button_ = new QPushButton(tr("Add image overlay"), overlay_body);
  overlay_layout->addWidget(add_overlay_button_);
  connect(add_overlay_button_, &QPushButton::clicked, this,
          &ImageSettingsWidget::addOverlayRequested);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Image overlays"), overlay_body, true));

  auto* annotation_body = new QWidget(this);
  annotation_list_layout_ = new QVBoxLayout(annotation_body);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Image annotations"), annotation_body, true));

  auto* marker_body = new QWidget(this);
  marker_list_layout_ = new QVBoxLayout(marker_body);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("3D markers"), marker_body, false));

  auto* scene_body = new QWidget(this);
  auto* scene_form = new QFormLayout(scene_body);
  ApplyCompactForm(scene_form);
  label_scale_spin_ = new QDoubleSpinBox(scene_body);
  label_scale_spin_->setRange(0.1, 8.0);
  label_scale_spin_->setSingleStep(0.1);
  label_scale_spin_->setValue(config_.label_scale);
  scene_form->addRow(tr("Label scale"), label_scale_spin_);
  background_edit_ = new QLineEdit(config_.background_color.name(), scene_body);
  scene_form->addRow(tr("Background"), background_edit_);
  outer->addWidget(MakeCollapsibleSection(this, tr("Scene"), scene_body, false));

  auto* publish_body = new QWidget(this);
  auto* publish_form = new QFormLayout(publish_body);
  ApplyCompactForm(publish_form);
  click_topic_edit_ = new QLineEdit(publish_body);
  click_topic_edit_->setPlaceholderText(QStringLiteral("/foxglove/cursor/click"));
  hover_topic_edit_ = new QLineEdit(publish_body);
  hover_topic_edit_->setPlaceholderText(QStringLiteral("/foxglove/cursor/hover"));
  publish_form->addRow(tr("Click topic"), click_topic_edit_);
  publish_form->addRow(tr("Hover topic"), hover_topic_edit_);
  outer->addWidget(MakeCollapsibleSection(this, tr("Publish"), publish_body, false));

  outer->addStretch();

  connect(topic_combo_, &QComboBox::currentTextChanged, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(calibration_combo_, &QComboBox::currentTextChanged, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(strict_sync_check_, &QCheckBox::toggled, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(undistort_check_, &QCheckBox::toggled, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(flip_h_check_, &QCheckBox::toggled, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(flip_v_check_, &QCheckBox::toggled, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(rotation_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(color_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(color_min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(color_max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(label_scale_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(background_edit_, &QLineEdit::textChanged, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(click_topic_edit_, &QLineEdit::textChanged, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(hover_topic_edit_, &QLineEdit::textChanged, this,
          &ImageSettingsWidget::emitConfigChanged);

  refreshChannelLists();
  setConfig(config_);
}

QStringList ImageSettingsWidget::imageChannels() const {
  QStringList channels;
  if (manager_ == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (display::isImageMessageType(info.message_type)) {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

QStringList ImageSettingsWidget::calibrationChannels() const {
  QStringList channels;
  if (manager_ == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (info.message_type == "automsgs.msgs.sensor_msgs.CameraInfo" ||
        info.message_type == "sensor_msgs/CameraInfo") {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

QStringList ImageSettingsWidget::annotationChannels() const {
  QStringList channels;
  if (manager_ == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (info.message_type.find("vision_msgs") != std::string::npos ||
        info.message_type.find("ImageAnnotations") != std::string::npos) {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

QStringList ImageSettingsWidget::markerChannels() const {
  QStringList channels;
  if (manager_ == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (info.message_type.find("visualization_msgs") != std::string::npos ||
        info.message_type.find("Marker") != std::string::npos) {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

void ImageSettingsWidget::refreshChannelLists() {
  const QString current_topic = topic_combo_->currentText();
  const QString current_calibration = calibration_combo_->currentText();
  topic_combo_->blockSignals(true);
  calibration_combo_->blockSignals(true);
  topic_combo_->clear();
  calibration_combo_->clear();
  calibration_combo_->addItem(QString(), QString());
  for (const QString& channel : imageChannels()) {
    topic_combo_->addItem(channel);
  }
  for (const QString& channel : calibrationChannels()) {
    calibration_combo_->addItem(channel);
  }
  topic_combo_->setCurrentText(current_topic);
  calibration_combo_->setCurrentText(current_calibration);
  topic_combo_->blockSignals(false);
  calibration_combo_->blockSignals(false);
  rebuildAnnotationSection();
  rebuildMarkerSection();
}

void ImageSettingsWidget::rebuildMarkerSection() {
  while (QLayoutItem* item = marker_list_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  const QStringList available = markerChannels();
  for (const QString& channel : available) {
    auto* check = new QCheckBox(channel, this);
    check->setChecked(config_.marker_channels.contains(channel));
    marker_list_layout_->addWidget(check);
    connect(check, &QCheckBox::toggled, this, &ImageSettingsWidget::emitConfigChanged);
  }
  if (available.isEmpty()) {
    marker_list_layout_->addWidget(
        new QLabel(tr("No marker topics available"), this));
  }
}

void ImageSettingsWidget::rebuildOverlaySection() {
  while (QLayoutItem* item = overlay_list_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  for (int i = 0; i < config_.overlays.size(); ++i) {
    const ImageOverlayConfig& overlay = config_.overlays.at(i);
    auto* row = new QWidget(this);
    auto* layout = new QFormLayout(row);
    auto* topic = new QComboBox(row);
    topic->setEditable(true);
    for (const QString& channel : imageChannels()) {
      topic->addItem(channel);
    }
    topic->setCurrentText(overlay.channel);
    layout->addRow(tr("Topic"), topic);
    auto* opacity = new QDoubleSpinBox(row);
    opacity->setRange(0.0, 1.0);
    opacity->setSingleStep(0.05);
    opacity->setValue(overlay.opacity);
    layout->addRow(tr("Opacity"), opacity);
    auto* blend = new QComboBox(row);
    blend->addItem(tr("Alpha"), static_cast<int>(ImageBlendMode::kAlpha));
    blend->addItem(tr("Add"), static_cast<int>(ImageBlendMode::kAdd));
    blend->setCurrentIndex(overlay.blend_mode == ImageBlendMode::kAdd ? 1 : 0);
    layout->addRow(tr("Blend mode"), blend);
    auto* pixel_alpha = new QComboBox(row);
    pixel_alpha->addItem(tr("None"), static_cast<int>(ImagePixelAlpha::kNone));
    pixel_alpha->addItem(tr("White transparent"),
                         static_cast<int>(ImagePixelAlpha::kWhiteTransparent));
    pixel_alpha->setCurrentIndex(overlay.pixel_alpha == ImagePixelAlpha::kWhiteTransparent
                                     ? 1
                                     : 0);
    layout->addRow(tr("Pixel alpha"), pixel_alpha);
    auto* remove = new QPushButton(tr("Remove overlay"), row);
    layout->addRow(QString(), remove);
    overlay_list_layout_->addWidget(row);

    connect(topic, &QComboBox::currentTextChanged, this,
            &ImageSettingsWidget::emitConfigChanged);
    connect(opacity, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &ImageSettingsWidget::emitConfigChanged);
    connect(blend, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &ImageSettingsWidget::emitConfigChanged);
    connect(pixel_alpha, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &ImageSettingsWidget::emitConfigChanged);
    connect(remove, &QPushButton::clicked, this, [this, i]() {
      emit removeOverlayRequested(i);
    });
  }
}

void ImageSettingsWidget::rebuildAnnotationSection() {
  while (QLayoutItem* item = annotation_list_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  const QStringList available = annotationChannels();
  for (const QString& channel : available) {
    auto* check = new QCheckBox(channel, this);
    check->setChecked(config_.annotation_channels.contains(channel));
    annotation_list_layout_->addWidget(check);
    connect(check, &QCheckBox::toggled, this, &ImageSettingsWidget::emitConfigChanged);
  }
  if (available.isEmpty()) {
    annotation_list_layout_->addWidget(
        new QLabel(tr("No annotation topics available"), this));
  }
}

ImagePanelConfig ImageSettingsWidget::config() const {
  ImagePanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.image_channel = topic_combo_->currentText().trimmed();
  out.calibration_channel = calibration_combo_->currentText().trimmed();
  out.strict_time_sync = strict_sync_check_->isChecked();
  out.enable_undistort = undistort_check_->isChecked();
  out.flip_horizontal = flip_h_check_->isChecked();
  out.flip_vertical = flip_v_check_->isChecked();
  out.rotation = static_cast<ImageRotation>(rotation_combo_->currentData().toInt());
  out.color_mode = static_cast<ImageColorMode>(color_mode_combo_->currentData().toInt());
  out.color_min = color_min_spin_->value();
  out.color_max = color_max_spin_->value();
  out.label_scale = label_scale_spin_->value();
  out.background_color = QColor(background_edit_->text().trimmed());
  if (!out.background_color.isValid()) {
    out.background_color = Qt::black;
  }
  out.click_publish_channel = click_topic_edit_->text().trimmed();
  out.hover_publish_channel = hover_topic_edit_->text().trimmed();

  out.overlays.clear();
  for (int i = 0; i < overlay_list_layout_->count(); ++i) {
    auto* row = qobject_cast<QWidget*>(overlay_list_layout_->itemAt(i)->widget());
    if (row == nullptr) {
      continue;
    }
    const auto combos = row->findChildren<QComboBox*>();
    const auto spins = row->findChildren<QDoubleSpinBox*>();
    if (combos.size() < 3 || spins.isEmpty()) {
      continue;
    }
    ImageOverlayConfig overlay;
    overlay.channel = combos.at(0)->currentText().trimmed();
    overlay.opacity = spins.first()->value();
    overlay.blend_mode =
        static_cast<ImageBlendMode>(combos.at(1)->currentData().toInt());
    overlay.pixel_alpha =
        static_cast<ImagePixelAlpha>(combos.at(2)->currentData().toInt());
    overlay.enabled = !overlay.channel.isEmpty();
    out.overlays.push_back(overlay);
  }

  out.annotation_channels.clear();
  for (int i = 0; i < annotation_list_layout_->count(); ++i) {
    auto* check = qobject_cast<QCheckBox*>(annotation_list_layout_->itemAt(i)->widget());
    if (check != nullptr && check->isChecked()) {
      out.annotation_channels.push_back(check->text());
    }
  }

  out.marker_channels.clear();
  for (int i = 0; i < marker_list_layout_->count(); ++i) {
    auto* check = qobject_cast<QCheckBox*>(marker_list_layout_->itemAt(i)->widget());
    if (check != nullptr && check->isChecked()) {
      out.marker_channels.push_back(check->text());
    }
  }
  return out;
}

void ImageSettingsWidget::setConfig(const ImagePanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  topic_combo_->setCurrentText(config_.image_channel);
  calibration_combo_->setCurrentText(config_.calibration_channel);
  strict_sync_check_->setChecked(config_.strict_time_sync);
  undistort_check_->setChecked(config_.enable_undistort);
  flip_h_check_->setChecked(config_.flip_horizontal);
  flip_v_check_->setChecked(config_.flip_vertical);
  rotation_combo_->setCurrentIndex(
      rotation_combo_->findData(static_cast<int>(config_.rotation)));
  color_mode_combo_->setCurrentIndex(
      color_mode_combo_->findData(static_cast<int>(config_.color_mode)));
  color_min_spin_->setValue(config_.color_min);
  color_max_spin_->setValue(config_.color_max);
  label_scale_spin_->setValue(config_.label_scale);
  background_edit_->setText(config_.background_color.name());
  click_topic_edit_->setText(config_.click_publish_channel);
  hover_topic_edit_->setText(config_.hover_publish_channel);
  rebuildOverlaySection();
  rebuildAnnotationSection();
  rebuildMarkerSection();
}

void ImageSettingsWidget::emitConfigChanged() {
  config_ = config();
  emit configChanged();
}

}  // namespace image
}  // namespace autoviz
