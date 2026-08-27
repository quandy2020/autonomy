/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <cstdint>
#include <string>

#include "autoviz/integration/message_queue.hpp"

class QComboBox;
class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QFrame;
class QLabel;
class QLineEdit;
class QMimeData;
class QTimer;

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace raw_messages {
class RawMessageTreeWidget;
}  // namespace raw_messages

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
  void selectChannel(const QString& channel);
  /** Re-render the last payload after global variables change. */
  void refreshFromVariables();

 signals:
  /** Request adding a numeric field as a Plot series (drag target / context menu). */
  void addToPlotRequested(const QString& channel, const QString& field_path);

 protected:
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void onChannelChanged(int index);
  void onMessagePathEdited();
  void onTick();

 private:
  void applyChromeStyles();
  void resubscribe();
  void unsubscribe();
  void clearSelection();
  void tryResubscribeIfNeeded();
  bool channelsStructureChanged();
  void rebuildChannelCombo();
  void updateSchemaHeader();
  void updateStatusChip(const QString& text);
  void showSchemaPlaceholder();
  void showPayload(const std::string& payload);
  void renderMessage(const google::protobuf::Message& message);
  std::string messageTypeForChannel(const std::string& channel) const;
  bool acceptChannelDrop(const QMimeData* mime) const;
  QString resolvedMessagePath() const;

  common::VisualizationManager* manager_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QLineEdit* message_path_edit_ = nullptr;
  QLabel* schema_badge_ = nullptr;
  QLabel* schema_label_ = nullptr;
  QLabel* status_label_ = nullptr;
  QFrame* empty_hint_ = nullptr;
  raw_messages::RawMessageTreeWidget* message_tree_ = nullptr;
  QTimer* tick_timer_ = nullptr;
  integration::MessageQueue payload_queue_;
  QStringList cached_channel_keys_;
  std::uint64_t subscription_id_ = 0;
  std::string active_channel_;
  std::string last_payload_;
  std::string last_rendered_payload_;
  bool message_tree_seeded_ = false;
  QString last_tree_root_label_;
  QString last_tree_path_filter_;
};

}  // namespace autoviz
