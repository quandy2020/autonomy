/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/strata_floor_panel.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QShowEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <automsgs/msgs/strata_msgs/floor_info.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {

StrataFloorPanel::StrataFloorPanel(common::VisualizationManager* manager,
                                   QWidget* parent)
    : QWidget(parent), manager_(manager) {
  setupUi();
}

void StrataFloorPanel::setupUi() {
  ApplyPanelShell(this);
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  QHBoxLayout* toolbar_layout = nullptr;
  auto* toolbar = MakePanelToolbar(this, &toolbar_layout);
  auto* title = new QLabel(tr("Strata Floors"), toolbar);
  StyleSectionTitle(title);
  toolbar_layout->addWidget(title, 1);
  layout->addWidget(toolbar);

  auto* body = new QWidget(this);
  auto* body_layout = new QVBoxLayout(body);
  ApplyCompactVBox(body_layout);
  active_label_ = new QLabel(tr("Active floor: —"), body);
  StyleHintLabel(active_label_);
  floor_selector_ = new QComboBox(body);
  floor_selector_->setEnabled(false);
  body_layout->addWidget(active_label_);
  body_layout->addWidget(floor_selector_);
  body_layout->addStretch();
  layout->addWidget(body, 1);
  connect(floor_selector_, qOverload<int>(&QComboBox::activated), this,
          &StrataFloorPanel::onFloorSelected);

  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, &StrataFloorPanel::pollMessages);
  // Timer is started lazily in showEvent() to avoid subscribing /strata/floors
  // until the panel is actually visible.
}

void StrataFloorPanel::setFloorsChannel(const std::string& channel) {
  floors_channel_ = channel;
  readers_ready_ = false;
}

void StrataFloorPanel::setSwitchChannel(const std::string& channel) {
  switch_channel_ = channel;
  switch_writer_.reset();
}

void StrataFloorPanel::ensureReaders() {
  if (readers_ready_ || manager_ == nullptr || floors_channel_.empty()) {
    return;
  }
  const auto node = manager_->autolinkNode();
  if (node == nullptr) {
    return;
  }
  floors_reader_ = node->CreateReader<autolink::message::RawMessage>(
      floors_channel_,
      [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
        if (msg != nullptr) {
          latest_floors_payload_ = msg->message;
        }
      });
  readers_ready_ = floors_reader_ != nullptr;
}

void StrataFloorPanel::pollMessages() {
  ensureReaders();
  if (latest_floors_payload_.empty()) {
    return;
  }

  automsgs::msgs::strata_msgs::FloorInfoArray message;
  if (!message.ParseFromString(latest_floors_payload_)) {
    return;
  }

  std::vector<FloorEntry> floors;
  floors.reserve(static_cast<size_t>(message.floors_size()));
  for (const auto& floor : message.floors()) {
    FloorEntry entry;
    entry.id = floor.id();
    entry.name = floor.name().empty() ? floor.id() : floor.name();
    floors.push_back(std::move(entry));
  }
  rebuildFloorList(floors, message.active_floor_id());
}

void StrataFloorPanel::rebuildFloorList(const std::vector<FloorEntry>& floors,
                                        const std::string& active_floor_id) {
  if (floors.empty()) {
    floor_selector_->clear();
    floor_selector_->setEnabled(false);
    active_label_->setText(tr("Active floor: —"));
    floors_.clear();
    active_floor_id_.clear();
    return;
  }

  const bool same_list = floors.size() == floors_.size() &&
                         active_floor_id == active_floor_id_;
  if (same_list) {
    bool unchanged = true;
    for (size_t i = 0; i < floors.size(); ++i) {
      if (floors[i].id != floors_[i].id || floors[i].name != floors_[i].name) {
        unchanged = false;
        break;
      }
    }
    if (unchanged) {
      return;
    }
  }

  floors_ = floors;
  active_floor_id_ = active_floor_id;
  updating_selection_ = true;
  floor_selector_->clear();
  int active_index = -1;
  for (size_t i = 0; i < floors_.size(); ++i) {
    floor_selector_->addItem(QString::fromStdString(floors_[i].name),
                             QString::fromStdString(floors_[i].id));
    if (floors_[i].id == active_floor_id_) {
      active_index = static_cast<int>(i);
    }
  }
  if (active_index >= 0) {
    floor_selector_->setCurrentIndex(active_index);
  }
  floor_selector_->setEnabled(true);
  updating_selection_ = false;
  active_label_->setText(
      tr("Active floor: %1").arg(QString::fromStdString(active_floor_id_)));
}

void StrataFloorPanel::onFloorSelected(int index) {
  if (updating_selection_ || index < 0 || index >= floor_selector_->count()) {
    return;
  }
  const std::string floor_id = floor_selector_->itemData(index).toString().toStdString();
  if (floor_id.empty() || floor_id == active_floor_id_) {
    return;
  }
  publishFloorSwitch(floor_id);
}

void StrataFloorPanel::publishFloorSwitch(const std::string& floor_id) {
  if (manager_ == nullptr || switch_channel_.empty()) {
    return;
  }
  const auto node = manager_->autolinkNode();
  if (node == nullptr) {
    return;
  }
  if (!switch_writer_) {
    switch_writer_ = node->CreateWriter<autolink::message::RawMessage>(switch_channel_);
  }
  if (!switch_writer_) {
    return;
  }
  automsgs::msgs::std_msgs::String message;
  message.set_data(floor_id);
  auto raw = std::make_shared<autolink::message::RawMessage>();
  message.SerializeToString(&raw->message);
  switch_writer_->Write(raw);
}

void StrataFloorPanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (poll_timer_ != nullptr && !poll_timer_->isActive()) {
    poll_timer_->start(100);
  }
}

void StrataFloorPanel::hideEvent(QHideEvent* event) {
  QWidget::hideEvent(event);
  if (poll_timer_ != nullptr) {
    poll_timer_->stop();
  }
}

}  // namespace autoviz
