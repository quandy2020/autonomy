/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QWidget>

class QComboBox;
class QHideEvent;
class QLabel;
class QShowEvent;
class QTimer;

namespace autolink {
template <typename MessageT>
class Reader;
template <typename MessageT>
class Writer;
namespace message {
class RawMessage;
}  // namespace message
}  // namespace autolink

namespace autoviz {
namespace common {
class VisualizationManager;
}

class StrataFloorPanel : public QWidget {
  Q_OBJECT

 public:
  explicit StrataFloorPanel(common::VisualizationManager* manager,
                            QWidget* parent = nullptr);

  void setFloorsChannel(const std::string& channel);
  void setSwitchChannel(const std::string& channel);

 protected:
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private slots:
  void onFloorSelected(int index);
  void pollMessages();

 private:
  struct FloorEntry {
    std::string id;
    std::string name;
  };

  void setupUi();
  void ensureReaders();
  void rebuildFloorList(const std::vector<FloorEntry>& floors,
                        const std::string& active_floor_id);
  void publishFloorSwitch(const std::string& floor_id);

  common::VisualizationManager* manager_ = nullptr;
  std::string floors_channel_{"/strata/floors"};
  std::string switch_channel_{"/strata/floor_switch"};
  std::string latest_floors_payload_;
  std::vector<FloorEntry> floors_;
  std::string active_floor_id_;
  bool updating_selection_{false};
  bool readers_ready_{false};

  QTimer* poll_timer_ = nullptr;
  QLabel* active_label_ = nullptr;
  QComboBox* floor_selector_ = nullptr;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> floors_reader_;
  std::shared_ptr<autolink::Writer<autolink::message::RawMessage>> switch_writer_;
};

}  // namespace autoviz
