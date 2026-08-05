/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QPointer>
#include <QSet>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/log/log_types.hpp"

class QLineEdit;
class QLabel;
class QScrollArea;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace log_panel {
class LogSettingsWidget;
class LogViewWidget;

class LogPanel : public QWidget {
  Q_OBJECT

 public:
  explicit LogPanel(common::VisualizationManager* manager, QWidget* parent = nullptr);
  ~LogPanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  LogPanelConfig config() const;
  void setConfig(const LogPanelConfig& config);
  void cloneConfigFrom(const LogPanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshSettingsChannels();

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

 private slots:
  void onToggleSettings(bool visible);
  void onHubLogAppended(const LogEntry& entry);
  void onChannelPayload(const std::string& payload);
  void onFollowTick();
  void onSearchChanged(const QString& text);
  void onEntryClicked(const LogEntry& entry);
  void onEntryHovered(const LogEntry& entry);
  void onStatsChanged(int visible_count, int total_count, bool pinned);

 private:
  void updateStatusBar(int visible_count, int total_count, bool pinned);
  void ingestEntry(const LogEntry& entry);
  void resubscribeTopic();
  void unsubscribeTopic();
  void applyConfigToUi();
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();
  void updateNamespaceUi();
  QStringList parseSearchTerms(const QString& text) const;
  QSet<QString> enabledNamespacesFromSettings() const;

  common::VisualizationManager* manager_ = nullptr;
  LogPanelConfig config_;
  LogViewWidget* view_ = nullptr;
  QLineEdit* search_edit_ = nullptr;
  QLabel* status_label_ = nullptr;
  QToolButton* search_clear_button_ = nullptr;
  LogSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
  QTimer* follow_timer_ = nullptr;
  QSet<QString> known_namespaces_;
  integration::ChannelReaderRegistry::SubscriptionId topic_subscription_id_ = 0;
};

}  // namespace log_panel
}  // namespace autoviz
