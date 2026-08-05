/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QPointer>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/table/table_types.hpp"

class QLabel;
class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QMimeData;
class QScrollArea;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace table {

class TableSettingsWidget;
class TableViewWidget;

class TablePanel : public QWidget {
  Q_OBJECT

 public:
  explicit TablePanel(common::VisualizationManager* manager, QWidget* parent = nullptr);
  ~TablePanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  TablePanelConfig config() const;
  void setConfig(const TablePanelConfig& config);
  void cloneConfigFrom(const TablePanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshSettingsChannels();

  void handleTableDrop(const QString& channel, const QString& array_path);

 signals:
  void configChanged();
  void activated();
  void settingsToggled(bool visible);
  void panelSplitRequested(Qt::Orientation orientation);
  void panelExpandRequested();
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void onToggleSettings(bool visible);
  void onChannelPayload(const std::string& payload);

 private:
  void resubscribeChannel();
  void unsubscribeChannel();
  void applyConfigToUi();
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();
  void updateStatusBar();
  void ingestPayload(const std::string& payload);
  QString messageTypeForChannel(const QString& channel) const;
  bool readDropPayload(const QMimeData* mime, QString* channel,
                       QString* array_path) const;

  common::VisualizationManager* manager_ = nullptr;
  TablePanelConfig config_;
  TableViewWidget* view_ = nullptr;
  int last_row_count_ = 0;
  QLabel* status_label_ = nullptr;
  TableSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
  integration::ChannelReaderRegistry::SubscriptionId subscription_id_ = 0;
  std::string subscribed_message_type_;
};

}  // namespace table
}  // namespace autoviz
