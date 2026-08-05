/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <cstdint>
#include <string>

class QComboBox;
class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QLineEdit;
class QMimeData;
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
  void selectChannel(const QString& channel);
  /** Re-render the last payload after global variables change. */
  void refreshFromVariables();

 protected:
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void onChannelChanged(int index);
  void onMessagePathEdited();

 private:
  void resubscribe();
  void unsubscribe();
  void showPayload(const std::string& payload);
  std::string messageTypeForChannel(const std::string& channel) const;
  bool acceptChannelDrop(const QMimeData* mime) const;
  QString resolvedMessagePath() const;

  common::VisualizationManager* manager_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QLineEdit* message_path_edit_ = nullptr;
  QPlainTextEdit* content_ = nullptr;
  std::uint64_t subscription_id_ = 0;
  std::string active_channel_;
  std::string last_payload_;
};

}  // namespace autoviz
