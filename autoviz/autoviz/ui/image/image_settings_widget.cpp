/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_settings_widget.hpp"

#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QShowEvent>
#include <QSignalBlocker>
#include <QHash>
#include <QVBoxLayout>

#include <algorithm>
#include <functional>
#include <string>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace image {
namespace {

QLabel* MakeFormLabel(const QString& text, QWidget* parent) {
  return MakeSettingsFormLabel(text, parent);
}

void StylePlainSpin(QDoubleSpinBox* spin) {
  if (spin == nullptr) {
    return;
  }
  spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spin->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  StyleCompactSettingsField(spin);
}

void StyleSettingsCombo(QComboBox* combo) {
  StyleCompactSettingsField(combo);
}

QString SoftCardStyle(const QString& object_name) {
  QHash<QString, QString> tokens = style::tokens();
  tokens.insert(QStringLiteral("{{name}}"), object_name);
  return style::sheet(QStringLiteral("image/overlay_card"), tokens);
}

/** Click anywhere on the field to open the channel list.
 *  The app theme draws QComboBox like a line edit and only the (invisible)
 *  arrow hit-target calls showPopup, so a normal click appears to do nothing.
 */
class ChannelPickCombo : public QComboBox {
 public:
  explicit ChannelPickCombo(QWidget* parent) : QComboBox(parent) {
    setEditable(false);
    setFocusPolicy(Qt::StrongFocus);
    setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    setMinimumContentsLength(12);
    setMaxVisibleItems(24);
    StyleCompactSettingsField(this);
  }

  std::function<void()> before_popup;

  void mousePressEvent(QMouseEvent* event) override {
    if (event->button() == Qt::LeftButton) {
      if (before_popup) {
        before_popup();
      }
      QComboBox::showPopup();
      if (view() != nullptr) {
        view()->setMinimumWidth(std::max(width(), 280));
      }
      event->accept();
      return;
    }
    QComboBox::mousePressEvent(event);
  }

  void mouseReleaseEvent(QMouseEvent* event) override {
    // Default release hides a popup that was opened on press, so the list
    // never stays on screen.
    event->accept();
  }
};

}  // namespace

ImageSettingsWidget::ImageSettingsWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : QWidget(parent), manager_(manager), config_() {
  ApplyCompactSettingsShell(this);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(4, 4, 4, 4);
  outer->setSpacing(4);
  outer->setAlignment(Qt::AlignTop);

  // —— General (includes title) ——
  auto* general_body = new QWidget(this);
  auto* general_form = new QFormLayout(general_body);
  ApplyCompactForm(general_form);

  title_edit_ = new QLineEdit(config_.title, general_body);
  title_edit_->setPlaceholderText(tr("Image"));
  StyleCompactSettingsField(title_edit_);
  general_form->addRow(MakeFormLabel(tr("Title"), general_body), title_edit_);

  auto* channel_pick = new ChannelPickCombo(general_body);
  channel_pick->before_popup = [this]() { refreshImageChannelItems(); };
  channel_combo_ = channel_pick;
  general_form->addRow(MakeFormLabel(tr("Channel"), general_body),
                       channel_combo_);

  calibration_combo_ = new QComboBox(general_body);
  calibration_combo_->setEditable(true);
  calibration_combo_->setPlaceholderText(tr("Optional"));
  calibration_combo_->addItem(QString(), QString());
  StyleSettingsCombo(calibration_combo_);
  general_form->addRow(MakeFormLabel(tr("Calibration"), general_body),
                       calibration_combo_);

  auto make_toggle = [general_body](const QString& tip) {
    auto* check = new QCheckBox(general_body);
    check->setStyleSheet(
        style::sheet(QStringLiteral("image/settings_check")));
    check->setToolTip(tip);
    check->setCursor(Qt::PointingHandCursor);
    return check;
  };

  strict_sync_check_ = make_toggle(tr("Require matching timestamps with calibration"));
  general_form->addRow(MakeFormLabel(tr("Strict time sync"), general_body),
                       strict_sync_check_);

  undistort_check_ = make_toggle(tr("Undistort using camera calibration"));
  general_form->addRow(MakeFormLabel(tr("Undistort image"), general_body),
                       undistort_check_);

  flip_h_check_ = make_toggle(tr("Flip image horizontally"));
  general_form->addRow(MakeFormLabel(tr("Flip horizontal"), general_body),
                       flip_h_check_);

  flip_v_check_ = make_toggle(tr("Flip image vertically"));
  general_form->addRow(MakeFormLabel(tr("Flip vertical"), general_body),
                       flip_v_check_);

  rotation_combo_ = new QComboBox(general_body);
  rotation_combo_->addItem(tr("0°"), static_cast<int>(ImageRotation::k0));
  rotation_combo_->addItem(tr("90°"), static_cast<int>(ImageRotation::k90));
  rotation_combo_->addItem(tr("180°"), static_cast<int>(ImageRotation::k180));
  rotation_combo_->addItem(tr("270°"), static_cast<int>(ImageRotation::k270));
  StyleSettingsCombo(rotation_combo_);
  general_form->addRow(MakeFormLabel(tr("Rotation"), general_body),
                       rotation_combo_);

  color_mode_combo_ = new QComboBox(general_body);
  color_mode_combo_->addItem(tr("Off"), static_cast<int>(ImageColorMode::kOff));
  color_mode_combo_->addItem(tr("Turbo"),
                             static_cast<int>(ImageColorMode::kTurbo));
  color_mode_combo_->addItem(tr("Rainbow"),
                             static_cast<int>(ImageColorMode::kRainbow));
  color_mode_combo_->addItem(tr("Grayscale"),
                             static_cast<int>(ImageColorMode::kGrayscale));
  StyleSettingsCombo(color_mode_combo_);
  general_form->addRow(MakeFormLabel(tr("Color mode"), general_body),
                       color_mode_combo_);

  color_min_spin_ = new QDoubleSpinBox(general_body);
  color_min_spin_->setRange(-1e9, 1e9);
  color_min_spin_->setDecimals(3);
  color_min_spin_->setValue(config_.color_min);
  StylePlainSpin(color_min_spin_);
  general_form->addRow(MakeFormLabel(tr("Value min"), general_body),
                       color_min_spin_);

  color_max_spin_ = new QDoubleSpinBox(general_body);
  color_max_spin_->setRange(-1e9, 1e9);
  color_max_spin_->setDecimals(3);
  color_max_spin_->setValue(config_.color_max);
  StylePlainSpin(color_max_spin_);
  general_form->addRow(MakeFormLabel(tr("Value max"), general_body),
                       color_max_spin_);

  outer->addWidget(
      MakeCollapsibleSection(this, tr("General"), general_body, true));

  // —— Image overlays ——
  auto* overlay_body = new QWidget(this);
  auto* overlay_layout = new QVBoxLayout(overlay_body);
  overlay_layout->setContentsMargins(0, 0, 0, 0);
  overlay_layout->setSpacing(4);
  overlay_list_layout_ = new QVBoxLayout();
  overlay_list_layout_->setSpacing(4);
  overlay_layout->addLayout(overlay_list_layout_);
  add_overlay_button_ =
      MakeFlatActionButton(tr("Add image overlay"), overlay_body);
  add_overlay_button_->setMinimumHeight(26);
  overlay_layout->addWidget(add_overlay_button_);
  connect(add_overlay_button_, &QPushButton::clicked, this,
          &ImageSettingsWidget::addOverlayRequested);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Image overlays"), overlay_body, false));

  // —— Annotations / markers ——
  auto* annotation_body = new QWidget(this);
  annotation_list_layout_ = new QVBoxLayout(annotation_body);
  annotation_list_layout_->setContentsMargins(0, 0, 0, 0);
  annotation_list_layout_->setSpacing(2);
  outer->addWidget(MakeCollapsibleSection(this, tr("Image annotations"),
                                          annotation_body, false));

  auto* marker_body = new QWidget(this);
  marker_list_layout_ = new QVBoxLayout(marker_body);
  marker_list_layout_->setContentsMargins(0, 0, 0, 0);
  marker_list_layout_->setSpacing(2);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("3D markers"), marker_body, false));

  auto* cloud_body = new QWidget(this);
  point_cloud_list_layout_ = new QVBoxLayout(cloud_body);
  point_cloud_list_layout_->setContentsMargins(0, 0, 0, 0);
  point_cloud_list_layout_->setSpacing(2);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Point clouds"), cloud_body, false));

  // —— Scene ——
  auto* scene_body = new QWidget(this);
  auto* scene_form = new QFormLayout(scene_body);
  ApplyCompactForm(scene_form);
  label_scale_spin_ = new QDoubleSpinBox(scene_body);
  label_scale_spin_->setRange(0.1, 8.0);
  label_scale_spin_->setSingleStep(0.1);
  label_scale_spin_->setDecimals(2);
  label_scale_spin_->setValue(config_.label_scale);
  StylePlainSpin(label_scale_spin_);
  scene_form->addRow(MakeFormLabel(tr("Label scale"), scene_body),
                     label_scale_spin_);

  auto* bg_row = new QWidget(scene_body);
  auto* bg_layout = new QHBoxLayout(bg_row);
  bg_layout->setContentsMargins(0, 0, 0, 0);
  bg_layout->setSpacing(6);
  background_edit_ = new QLineEdit(config_.background_color.name(), bg_row);
  background_edit_->setPlaceholderText(QStringLiteral("#000000"));
  StyleCompactSettingsField(background_edit_);
  auto* bg_pick = new QPushButton(tr("Pick"), bg_row);
  bg_pick->setCursor(Qt::PointingHandCursor);
  bg_pick->setFixedSize(40, 20);
  bg_pick->setStyleSheet(
      style::sheet(QStringLiteral("image/compact_pick_button")));
  bg_layout->addWidget(background_edit_, 1);
  bg_layout->addWidget(bg_pick, 0, Qt::AlignVCenter);
  scene_form->addRow(MakeFormLabel(tr("Background"), scene_body), bg_row);
  connect(bg_pick, &QPushButton::clicked, this, [this]() {
    const QColor current = QColor(background_edit_->text().trimmed());
    const QColor chosen = QColorDialog::getColor(
        current.isValid() ? current : Qt::black, this, tr("Background color"));
    if (!chosen.isValid()) {
      return;
    }
    background_edit_->setText(chosen.name(QColor::HexRgb));
  });
  outer->addWidget(MakeCollapsibleSection(this, tr("Scene"), scene_body, false));

  // —— Publish ——
  auto* publish_body = new QWidget(this);
  auto* publish_form = new QFormLayout(publish_body);
  ApplyCompactForm(publish_form);
  click_topic_edit_ = new QLineEdit(publish_body);
  click_topic_edit_->setPlaceholderText(
      QStringLiteral("/foxglove/cursor/click"));
  StyleCompactSettingsField(click_topic_edit_);
  hover_topic_edit_ = new QLineEdit(publish_body);
  hover_topic_edit_->setPlaceholderText(
      QStringLiteral("/foxglove/cursor/hover"));
  StyleCompactSettingsField(hover_topic_edit_);
  publish_form->addRow(MakeFormLabel(tr("Click topic"), publish_body),
                       click_topic_edit_);
  publish_form->addRow(MakeFormLabel(tr("Hover topic"), publish_body),
                       hover_topic_edit_);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Publish"), publish_body, false));

  outer->addStretch(1);

  connect(title_edit_, &QLineEdit::textChanged, this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(channel_combo_, &QComboBox::currentTextChanged, this,
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
  connect(rotation_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ImageSettingsWidget::emitConfigChanged);
  connect(color_mode_combo_,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &ImageSettingsWidget::emitConfigChanged);
  connect(color_min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &ImageSettingsWidget::emitConfigChanged);
  connect(color_max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &ImageSettingsWidget::emitConfigChanged);
  connect(label_scale_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
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
  if (manager_ != nullptr) {
    for (const integration::ChannelInfo& info : manager_->channels()) {
      if (display::isImageMessageType(info.message_type) ||
          info.channel_name.find("image") != std::string::npos ||
          info.channel_name.find("Image") != std::string::npos) {
        channels.push_back(QString::fromStdString(info.channel_name));
      }
    }
  }
  // Keep the active channel selectable even if discovery type is missing.
  if (!config_.image_channel.isEmpty() &&
      channels.indexOf(config_.image_channel) < 0) {
    channels.push_front(config_.image_channel);
  }
  // If type filter matched nothing, list all live channels so the dropdown
  // still works (non-image types fail at subscribe time).
  if (channels.isEmpty() && manager_ != nullptr) {
    for (const integration::ChannelInfo& info : manager_->channels()) {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  channels.removeDuplicates();
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

QStringList ImageSettingsWidget::calibrationChannels() const {
  QStringList channels;
  if (manager_ == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (commsgs::MessageTypesCompatible(
            info.message_type, "automsgs.msgs.sensor_msgs.CameraInfo")) {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  if (!config_.calibration_channel.isEmpty() &&
      channels.indexOf(config_.calibration_channel) < 0) {
    channels.push_front(config_.calibration_channel);
  }
  channels.removeDuplicates();
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

QStringList ImageSettingsWidget::pointCloudChannels() const {
  QStringList channels;
  if (manager_ == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (commsgs::MessageTypesCompatible(
            info.message_type, "automsgs.msgs.sensor_msgs.PointCloud2") ||
        info.message_type.find("PointCloud2") != std::string::npos) {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

void ImageSettingsWidget::rebuildPointCloudSection() {
  while (QLayoutItem* item = point_cloud_list_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  const QStringList available = pointCloudChannels();
  for (const QString& channel : available) {
    auto* check = new QCheckBox(channel, this);
    check->setChecked(config_.point_cloud_channels.contains(channel));
    check->setStyleSheet(
        style::sheet(QStringLiteral("image/settings_check")));
    point_cloud_list_layout_->addWidget(check);
    connect(check, &QCheckBox::toggled, this,
            &ImageSettingsWidget::emitConfigChanged);
  }
  if (available.isEmpty()) {
    auto* hint = new QLabel(tr("No PointCloud2 topics available"), this);
    hint->setStyleSheet(
        style::type(style::Role::Muted, 11, 400, true));
    point_cloud_list_layout_->addWidget(hint);
  }
}

void ImageSettingsWidget::refreshImageChannelItems() {
  if (channel_combo_ == nullptr) {
    return;
  }
  if (manager_ != nullptr) {
    manager_->refreshChannelList();
  }
  const QString current = channel_combo_->currentText().trimmed().isEmpty()
                              ? config_.image_channel
                              : channel_combo_->currentText();
  channel_combo_->blockSignals(true);
  channel_combo_->clear();
  const QStringList channels = imageChannels();
  if (channels.isEmpty()) {
    channel_combo_->addItem(tr("(no image channel)"));
    channel_combo_->setItemData(0, QString(), Qt::UserRole);
  } else {
    channel_combo_->addItems(channels);
  }
  const int index = channel_combo_->findText(current);
  if (index >= 0) {
    channel_combo_->setCurrentIndex(index);
  } else if (!current.isEmpty()) {
    channel_combo_->insertItem(0, current);
    channel_combo_->setCurrentIndex(0);
  }
  channel_combo_->blockSignals(false);
}

void ImageSettingsWidget::refreshChannelLists() {
  refreshImageChannelItems();
  const QString current_calibration = calibration_combo_->currentText();
  calibration_combo_->blockSignals(true);
  calibration_combo_->clear();
  calibration_combo_->addItem(QString(), QString());
  for (const QString& channel : calibrationChannels()) {
    calibration_combo_->addItem(channel);
  }
  calibration_combo_->setCurrentText(current_calibration);
  calibration_combo_->blockSignals(false);
  rebuildAnnotationSection();
  rebuildMarkerSection();
  rebuildPointCloudSection();
}

void ImageSettingsWidget::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  refreshImageChannelItems();
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
    check->setStyleSheet(
        style::sheet(QStringLiteral("image/settings_check")));
    marker_list_layout_->addWidget(check);
    connect(check, &QCheckBox::toggled, this,
            &ImageSettingsWidget::emitConfigChanged);
  }
  if (available.isEmpty()) {
    auto* hint = new QLabel(tr("No marker topics available"), this);
    hint->setStyleSheet(
        style::type(style::Role::Muted, 11, 400, true));
    marker_list_layout_->addWidget(hint);
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
    auto* row = new QFrame(this);
    row->setObjectName(QStringLiteral("ImageOverlayCard"));
    row->setStyleSheet(SoftCardStyle(QStringLiteral("ImageOverlayCard")));
    auto* layout = new QFormLayout(row);
    ApplyCompactForm(layout);
    layout->setContentsMargins(6, 4, 6, 4);

    auto* topic = new QComboBox(row);
    topic->setEditable(true);
    for (const QString& channel : imageChannels()) {
      topic->addItem(channel);
    }
    topic->setCurrentText(overlay.channel);
    layout->addRow(MakeFormLabel(tr("Topic"), row), topic);

    auto* opacity = new QDoubleSpinBox(row);
    opacity->setRange(0.0, 1.0);
    opacity->setSingleStep(0.05);
    opacity->setDecimals(2);
    opacity->setValue(overlay.opacity);
    StylePlainSpin(opacity);
    layout->addRow(MakeFormLabel(tr("Opacity"), row), opacity);

    auto* blend = new QComboBox(row);
    blend->addItem(tr("Alpha"), static_cast<int>(ImageBlendMode::kAlpha));
    blend->addItem(tr("Add"), static_cast<int>(ImageBlendMode::kAdd));
    blend->setCurrentIndex(overlay.blend_mode == ImageBlendMode::kAdd ? 1 : 0);
    layout->addRow(MakeFormLabel(tr("Blend"), row), blend);

    auto* pixel_alpha = new QComboBox(row);
    pixel_alpha->addItem(tr("None"), static_cast<int>(ImagePixelAlpha::kNone));
    pixel_alpha->addItem(tr("White transparent"),
                         static_cast<int>(ImagePixelAlpha::kWhiteTransparent));
    pixel_alpha->setCurrentIndex(
        overlay.pixel_alpha == ImagePixelAlpha::kWhiteTransparent ? 1 : 0);
    layout->addRow(MakeFormLabel(tr("Pixel alpha"), row), pixel_alpha);

    auto* actions = new QWidget(row);
    auto* actions_layout = new QHBoxLayout(actions);
    actions_layout->setContentsMargins(0, 0, 0, 0);
    actions_layout->setSpacing(4);
    auto* move_up = MakeFlatActionButton(tr("Up"), actions);
    auto* move_down = MakeFlatActionButton(tr("Down"), actions);
    auto* remove = MakeDestructiveFlatActionButton(tr("Remove"), actions);
    move_up->setEnabled(i > 0);
    move_down->setEnabled(i + 1 < config_.overlays.size());
    actions_layout->addWidget(move_up);
    actions_layout->addWidget(move_down);
    actions_layout->addWidget(remove);
    actions_layout->addStretch(1);
    layout->addRow(QString(), actions);
    overlay_list_layout_->addWidget(row);

    connect(topic, &QComboBox::currentTextChanged, this,
            &ImageSettingsWidget::emitConfigChanged);
    connect(opacity, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &ImageSettingsWidget::emitConfigChanged);
    connect(blend, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &ImageSettingsWidget::emitConfigChanged);
    connect(pixel_alpha, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ImageSettingsWidget::emitConfigChanged);
    connect(remove, &QPushButton::clicked, this, [this, i]() {
      emit removeOverlayRequested(i);
    });
    connect(move_up, &QPushButton::clicked, this, [this, i]() {
      emit moveOverlayRequested(i, -1);
    });
    connect(move_down, &QPushButton::clicked, this, [this, i]() {
      emit moveOverlayRequested(i, 1);
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
    check->setStyleSheet(
        style::sheet(QStringLiteral("image/settings_check")));
    annotation_list_layout_->addWidget(check);
    connect(check, &QCheckBox::toggled, this,
            &ImageSettingsWidget::emitConfigChanged);
  }
  if (available.isEmpty()) {
    auto* hint = new QLabel(tr("No annotation topics available"), this);
    hint->setStyleSheet(
        style::type(style::Role::Muted, 11, 400, true));
    annotation_list_layout_->addWidget(hint);
  }
}

ImagePanelConfig ImageSettingsWidget::config() const {
  ImagePanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  {
    const QString channel = channel_combo_->currentText().trimmed();
    out.image_channel =
        channel.startsWith(QLatin1Char('(')) ? QString() : channel;
  }
  out.calibration_channel = calibration_combo_->currentText().trimmed();
  out.strict_time_sync = strict_sync_check_->isChecked();
  out.enable_undistort = undistort_check_->isChecked();
  out.flip_horizontal = flip_h_check_->isChecked();
  out.flip_vertical = flip_v_check_->isChecked();
  out.rotation =
      static_cast<ImageRotation>(rotation_combo_->currentData().toInt());
  out.color_mode =
      static_cast<ImageColorMode>(color_mode_combo_->currentData().toInt());
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
    auto* check =
        qobject_cast<QCheckBox*>(annotation_list_layout_->itemAt(i)->widget());
    if (check != nullptr && check->isChecked()) {
      out.annotation_channels.push_back(check->text());
    }
  }

  out.marker_channels.clear();
  for (int i = 0; i < marker_list_layout_->count(); ++i) {
    auto* check =
        qobject_cast<QCheckBox*>(marker_list_layout_->itemAt(i)->widget());
    if (check != nullptr && check->isChecked()) {
      out.marker_channels.push_back(check->text());
    }
  }
  out.point_cloud_channels.clear();
  for (int i = 0; i < point_cloud_list_layout_->count(); ++i) {
    auto* check =
        qobject_cast<QCheckBox*>(point_cloud_list_layout_->itemAt(i)->widget());
    if (check != nullptr && check->isChecked()) {
      out.point_cloud_channels.push_back(check->text());
    }
  }
  return out;
}

void ImageSettingsWidget::setConfig(const ImagePanelConfig& config) {
  config_ = config;
  const QSignalBlocker b1(title_edit_);
  const QSignalBlocker b2(channel_combo_);
  const QSignalBlocker b3(calibration_combo_);
  const QSignalBlocker b4(strict_sync_check_);
  const QSignalBlocker b5(undistort_check_);
  const QSignalBlocker b6(flip_h_check_);
  const QSignalBlocker b7(flip_v_check_);
  const QSignalBlocker b8(rotation_combo_);
  const QSignalBlocker b9(color_mode_combo_);
  const QSignalBlocker b10(color_min_spin_);
  const QSignalBlocker b11(color_max_spin_);
  const QSignalBlocker b12(label_scale_spin_);
  const QSignalBlocker b13(background_edit_);
  const QSignalBlocker b14(click_topic_edit_);
  const QSignalBlocker b15(hover_topic_edit_);

  title_edit_->setText(config_.title);
  refreshImageChannelItems();
  {
    const int index = channel_combo_->findText(config_.image_channel);
    if (index >= 0) {
      channel_combo_->setCurrentIndex(index);
    }
  }
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
  rebuildPointCloudSection();
}

void ImageSettingsWidget::emitConfigChanged() {
  config_ = config();
  emit configChanged();
}

}  // namespace image
}  // namespace autoviz
