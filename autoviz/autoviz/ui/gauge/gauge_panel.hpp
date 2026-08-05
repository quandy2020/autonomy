/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QPointer>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/gauge/gauge_types.hpp"

class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QFrame;
class QLabel;
class QMimeData;
class QScrollArea;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace gauge {

class GaugeSettingsWidget;
class GaugeViewWidget;

class GaugePanel : public QWidget {
  Q_OBJECT

 public:
  explicit GaugePanel(common::VisualizationManager* manager, QWidget* parent = nullptr);
  ~GaugePanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  GaugePanelConfig config() const;
  void setConfig(const GaugePanelConfig& config);
  void cloneConfigFrom(const GaugePanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshSettingsChannels();

  void handleFieldDrop(const QString& channel, const QString& field_path);

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
                       QString* field_path) const;

  common::VisualizationManager* manager_ = nullptr;
  GaugePanelConfig config_;
  GaugeViewWidget* view_ = nullptr;
  QLabel* status_label_ = nullptr;
  GaugeSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
  integration::ChannelReaderRegistry::SubscriptionId subscription_id_ = 0;
  std::string subscribed_message_type_;
};

}  // namespace gauge
}  // namespace autoviz
