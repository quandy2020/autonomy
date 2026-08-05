/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <cstdint>
#include <string>

class QComboBox;
class QPlainTextEdit;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/** Foxglove-style raw message inspector for a selected Autolink channel. */
class RawMessagesPanel : public QWidget {
  Q_OBJECT

 public:
  explicit RawMessagesPanel(common::VisualizationManager* manager,
                            QWidget* parent = nullptr);
  ~RawMessagesPanel() override;

  void refreshChannels();

 private slots:
  void onChannelChanged(int index);

 private:
  void resubscribe();
  void unsubscribe();
  void showPayload(const std::string& payload);
  std::string messageTypeForChannel(const std::string& channel) const;

  common::VisualizationManager* manager_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QPlainTextEdit* content_ = nullptr;
  std::uint64_t subscription_id_ = 0;
  std::string active_channel_;
};

}  // namespace autoviz
