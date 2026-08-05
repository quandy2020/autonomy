/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QHash>
#include <QPointer>
#include <QTimer>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/map/map_layer_store.hpp"
#include "autoviz/ui/map/map_types.hpp"

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
namespace map {

class MapSettingsWidget;
class MapViewportWidget;

class MapPanel : public QWidget {
  Q_OBJECT

 public:
  explicit MapPanel(common::VisualizationManager* manager, QWidget* parent = nullptr);
  ~MapPanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  MapPanelConfig config() const;
  void setConfig(const MapPanelConfig& config);
  void cloneConfigFrom(const MapPanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshSettingsChannels();

  void handleChannelDrop(const QString& channel);

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
  void onFollowTick();
  void onViewChanged(double latitude, double longitude, double zoom);

 private:
  void resubscribeAll();
  void unsubscribeAll();
  void applyConfigToUi();
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();
  void updateStatusBar();
  void refreshViewport();
  void ingestChannelPayload(const QString& channel, const std::string& payload);
  QString messageTypeForChannel(const QString& channel) const;
  bool readDropPayload(const QMimeData* mime, QString* channel) const;
  void ensureTopicLayerForChannel(const QString& channel);
  quint64 nowNanoseconds() const;

  common::VisualizationManager* manager_ = nullptr;
  MapPanelConfig config_;
  MapLayerStore layer_store_;
  MapViewportWidget* view_ = nullptr;
  QLabel* status_label_ = nullptr;
  MapSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
  QTimer follow_timer_;
  QHash<QString, integration::ChannelReaderRegistry::SubscriptionId> subscriptions_;
  QHash<QString, std::string> subscribed_message_types_;
};

}  // namespace map
}  // namespace autoviz
