/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_panel.hpp
 * @brief Map panel — basemap tiles, topic layers, follow camera, and settings.
 *
 * Subscribes to configured topic channels, ingests payloads via
 * @ref MapMessageIngest into @ref MapLayerStore, and drives
 * @ref MapViewportWidget for Web Mercator rendering.
 *
 * ## Data flow
 *
 * - **In:** channel drops / settings → @c config_ → resubscribe → ingest →
 *   store snapshot → viewport.
 * - **Out:** @ref configChanged() for session persist; panel chrome signals.
 *
 * @see MapSettingsWidget
 * @see MapViewportWidget
 * @see MapPanelConfig
 */

#pragma once

#include <QWidget>

#include <QHash>
#include <QPointer>
#include <QTimer>
#include <QtGlobal>

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

/**
 * @class MapPanel
 * @brief Dockable 2D map panel with slippy tiles and live geo overlays.
 *
 * ## Layout
 *
 * @code
 * ┌──────────────────────────────────────────┐
 * │ [settings / expand title-bar tools]      │
 * ├──────────────────────────┬───────────────┤
 * │ MapViewportWidget        │ Settings      │
 * │ (tiles + layers)         │               │
 * ├──────────────────────────┤               │
 * │ status bar (lat/lon/z)   │               │
 * └──────────────────────────┴───────────────┘
 * @endcode
 *
 * @note Does not own @ref common::VisualizationManager.
 *
 * @see MapSettingsWidget
 * @see MapViewportWidget
 * @see MapLayerStore
 */
class MapPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the map view, settings UI, and follow timer.
   *
   * @param manager Non-owning visualization manager for channels / TF.
   * @param parent Qt parent widget.
   */
  explicit MapPanel(common::VisualizationManager* manager, QWidget* parent = nullptr);

  /**
   * @brief Unsubscribes all channels and stops timers.
   */
  ~MapPanel() override;

  /**
   * @brief Installs Settings / Expand tools on the dock title bar.
   *
   * @param dock Host dock widget.
   */
  void installTitleBarTools(PanelDockWidget* dock);

  /**
   * @brief Returns the current panel configuration.
   *
   * @return Copy of @c config_.
   */
  MapPanelConfig config() const;

  /**
   * @brief Replaces config, resubscribes, and refreshes the viewport.
   *
   * @param config Full map panel configuration.
   */
  void setConfig(const MapPanelConfig& config);

  /**
   * @brief Copies config for split/duplicate without carrying layer runtime.
   *
   * @param config Source configuration.
   */
  void cloneConfigFrom(const MapPanelConfig& config);

  /**
   * @brief Shows or hides the settings pane.
   *
   * @param visible @c true to show settings.
   */
  void setSettingsVisible(bool visible);

  /**
   * @brief Whether settings are currently visible.
   *
   * @return Settings visibility.
   */
  bool settingsVisible() const;

  /**
   * @brief Syncs the title-bar Settings button checked state.
   *
   * @param checked Desired checked state.
   */
  void setSettingsButtonChecked(bool checked);

  /**
   * @brief Syncs the title-bar Expand button checked state.
   *
   * @param checked Desired checked state.
   */
  void setExpandButtonChecked(bool checked);

  /**
   * @brief Returns the settings host for inspector reparenting.
   *
   * @return Settings container widget.
   * @see recallSettingsWidget()
   */
  QWidget* settingsWidgetForInspector();

  /**
   * @brief Reclaims settings from the shared inspector sidebar.
   */
  void recallSettingsWidget();

  /**
   * @brief Refreshes channel combos in the settings widget.
   */
  void refreshSettingsChannels();

  /**
   * @brief Ensures a topic layer exists for @p channel and selects it as needed.
   *
   * Entry point for external drops (Topics panel → Map).
   *
   * @param channel Channel name to add / focus.
   */
  void handleChannelDrop(const QString& channel);

 signals:
  /** Emitted when config should be persisted. */
  void configChanged();

  /** Emitted when this panel becomes active. */
  void activated();

  /**
   * @brief Emitted when settings visibility toggles.
   *
   * @param visible New visibility.
   */
  void settingsToggled(bool visible);

  /**
   * @brief Request splitting this panel.
   *
   * @param orientation Split orientation.
   */
  void panelSplitRequested(Qt::Orientation orientation);

  /** Request expanding this panel. */
  void panelExpandRequested();

  /** Request removing this panel. */
  void panelRemoveRequested();

  /**
   * @brief Request changing panel type by object name.
   *
   * @param object_name Target panel object name.
   */
  void panelChangeRequested(const QString& object_name);

 protected:
  /**
   * @brief Emits @ref activated() on focus.
   *
   * @param event Focus-in event.
   */
  void focusInEvent(QFocusEvent* event) override;

  /**
   * @brief Accepts map-compatible channel drag mime.
   *
   * @param event Drag-enter event.
   */
  void dragEnterEvent(QDragEnterEvent* event) override;

  /**
   * @brief Continues accepting map channel drags.
   *
   * @param event Drag-move event.
   */
  void dragMoveEvent(QDragMoveEvent* event) override;

  /**
   * @brief Adds a dropped channel as a topic layer.
   *
   * @param event Drop event.
   */
  void dropEvent(QDropEvent* event) override;

 private slots:
  /**
   * @brief Title-bar Settings toggle.
   *
   * @param visible Requested visibility.
   */
  void onToggleSettings(bool visible);

  /**
   * @brief Follow-timer tick: updates viewport center from follow target.
   */
  void onFollowTick();

  /**
   * @brief Viewport camera changed: writes back center/zoom into config.
   *
   * @param latitude Center latitude.
   * @param longitude Center longitude.
   * @param zoom Slippy-map zoom level.
   */
  void onViewChanged(double latitude, double longitude, double zoom);

 private:
  /** Rebuild all channel subscriptions from @c config_.topic_layers. */
  void resubscribeAll();

  /** Unsubscribe every active map channel. */
  void unsubscribeAll();

  /** Push config into viewport / status / settings widgets. */
  void applyConfigToUi();

  /** Mirror @c config_ into the settings form. */
  void syncSettingsWidgetFromConfig();

  /** Sync title-bar tool checked states. */
  void syncSettingsToolState();

  /** Refresh lat/lon/zoom status label. */
  void updateStatusBar();

  /** Push layer snapshots + follow target into the viewport. */
  void refreshViewport();

  /**
   * @brief Ingest a raw payload for a subscribed channel.
   *
   * @param channel Channel key.
   * @param payload Serialized message bytes.
   */
  void ingestChannelPayload(const QString& channel, const std::string& payload);

  /**
   * @brief Resolve message type for a channel via the manager.
   *
   * @param channel Channel name.
   * @return Message type string, or empty if unknown.
   */
  QString messageTypeForChannel(const QString& channel) const;

  /**
   * @brief Extracts a channel name from drag mime data.
   *
   * @param mime Drop mime payload.
   * @param channel Out-parameter for the channel name.
   * @return @c true when a channel was read.
   */
  bool readDropPayload(const QMimeData* mime, QString* channel) const;

  /**
   * @brief First local .geojson or .json path in a drop, if any.
   *
   * @param mime Drop mime payload.
   * @return Local path, or empty.
   */
  QString geoJsonPathFromMime(const QMimeData* mime) const;

  /**
   * @brief Ensures @c config_.topic_layers contains an entry for @p channel.
   *
   * @param channel Channel to add with default style if missing.
   */
  void ensureTopicLayerForChannel(const QString& channel);

  /**
   * @brief Current wall/sim time in nanoseconds for store filtering.
   *
   * @return Timestamp used by @ref MapLayerStore::snapshot().
   */
  quint64 nowNanoseconds() const;

  /** Non-owning visualization manager. */
  common::VisualizationManager* manager_ = nullptr;

  /** Persisted panel configuration. */
  MapPanelConfig config_;

  /** Thread-safe geo feature store. */
  MapLayerStore layer_store_;

  /** Slippy-map viewport. */
  MapViewportWidget* view_ = nullptr;

  /** Status bar showing center lat/lon/zoom. */
  QLabel* status_label_ = nullptr;

  /** Selection, measure, or GeoJSON error text from GeoMap. */
  QString map_status_extra_;

  /** Cursor lat/lon from the viewport; NaN when the mouse is outside. */
  double cursor_latitude_ = qQNaN();
  double cursor_longitude_ = qQNaN();

  /** Settings form. */
  MapSettingsWidget* settings_widget_ = nullptr;

  /** Scroll host for settings. */
  QScrollArea* settings_scroll_ = nullptr;

  /** Settings layout container inside the panel. */
  QWidget* settings_container_ = nullptr;

  /** Dock Settings button (weak). */
  QPointer<QToolButton> settings_button_;

  /** Dock Expand button (weak). */
  QPointer<QToolButton> expand_button_;

  /** Periodic follow-camera update timer. */
  QTimer follow_timer_;

  /** Channel → subscription id. */
  QHash<QString, integration::ChannelReaderRegistry::SubscriptionId> subscriptions_;

  /** Channel → message type at subscribe time (for ingest dispatch). */
  QHash<QString, std::string> subscribed_message_types_;
};

}  // namespace map
}  // namespace autoviz
