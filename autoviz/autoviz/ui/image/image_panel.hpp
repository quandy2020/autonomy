/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_panel.hpp
 * @brief Image panel — subscribe, decode, overlay, and display camera frames.
 *
 * Hosted in a @c PanelDockWidget. Subscribes to a main image channel plus
 * optional overlays, annotation, calibration (@c CameraInfo), and marker
 * channels; drains message queues on @ref tick() / frame timer and composites
 * into @ref ImageViewWidget.
 *
 * ## Data flow
 *
 * - **In:** channel payloads → decode / parse → @c base_image_ + overlay /
 *   annotation / marker runtimes → @ref updateRenderedFrame().
 * - **Out:** pixel click/hover publish; @ref configChanged() for session
 *   persistence; panel chrome signals for split / remove / expand.
 *
 * @see ImageSettingsWidget
 * @see ImageViewWidget
 * @see ImagePanelConfig
 * @see DisplayImageWindow
 */

#pragma once

#include <QWidget>

#include <memory>
#include <string>
#include <vector>

#include <QPointer>
#include <QVector>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/ui/image/image_analysis.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"
#include "autoviz/ui/image/image_marker_projection.hpp"
#include "autoviz/ui/image/image_types.hpp"
#include "autoviz/ui/image/image_video_decoder.hpp"

class QButtonGroup;
class QDragEnterEvent;
class QDoubleSpinBox;
class QFocusEvent;
class QImage;
class QLabel;
class QScrollArea;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace image {
class ImageHistogramWidget;
class ImageProfileWidget;
class ImageSettingsWidget;
class ImageViewWidget;
}

namespace image {

/**
 * @class ImagePanel
 * @brief Dockable panel that renders a live / recorded image stream with
 *        overlays, annotations, and projected markers.
 *
 * ## Layout
 *
 * @code
 * ┌──────────────────────────────────────────┐
 * │ ImageViewWidget (full bleed)             │
 * │                                          │
 * │   ┌─ floating glass chrome (bottom) ──┐  │
 * │   │ tools · hist/profile · hide       │  │
 * │   └───────────────────────────────────┘  │
 * └──────────────────────────────────────────┘
 * @endcode
 *
 * Settings may be reparented into the shared sidebar inspector via
 * @ref settingsWidgetForInspector() / @ref recallSettingsWidget().
 *
 * @note Does not own @ref common::VisualizationManager; lifetime is held by
 *       VisualizationFrame.
 *
 * @see ImageSettingsWidget
 * @see ImageViewWidget
 * @see image_config_io.hpp
 */
class ImagePanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel, builds the view / settings UI, and starts the
   *        frame timer.
   *
   * @param manager Non-owning visualization manager for channel discovery and
   *        subscriptions; may be @c nullptr until wired later via config apply.
   * @param parent Qt parent widget (typically dock content host).
   */
  explicit ImagePanel(common::VisualizationManager* manager,
                      QWidget* parent = nullptr);

  /**
   * @brief Unsubscribes all channels and tears down the frame timer.
   */
  ~ImagePanel() override;

  /**
   * @brief Installs Settings / Expand tool buttons into the dock title bar.
   *
   * @param dock Host dock widget that owns the title-bar tool area.
   */
  void installTitleBarTools(PanelDockWidget* dock);

  /**
   * @brief Returns the current panel configuration.
   *
   * @return Copy of @c config_.
   * @see setConfig()
   */
  ImagePanelConfig config() const;

  /**
   * @brief Replaces config, resubscribes channels, and refreshes UI.
   *
   * @param config Full panel configuration from session or settings edits.
   * @see applySettings()
   */
  void setConfig(const ImagePanelConfig& config);

  /**
   * @brief Copies config for split/duplicate; does not carry runtime frames.
   *
   * @param config Source panel config to clone.
   */
  void cloneConfigFrom(const ImagePanelConfig& config);

  /**
   * @brief Applies property edits from the settings widget without a full
   *        identity reset.
   *
   * @param config Edited config (typically from @ref ImageSettingsWidget).
   */
  void applySettings(const ImagePanelConfig& config);

  /**
   * @brief Shows or hides the embedded settings pane.
   *
   * @param visible @c true to show settings beside the view.
   * @see settingsVisible()
   */
  void setSettingsVisible(bool visible);

  /**
   * @brief Whether the settings pane is currently visible.
   *
   * @return @c true when settings UI is shown.
   */
  bool settingsVisible() const;

  /**
   * @brief Syncs the title-bar Settings button checked state.
   *
   * @param checked Desired button checked state.
   */
  void setSettingsButtonChecked(bool checked);

  /**
   * @brief Syncs the title-bar Expand button checked state.
   *
   * @param checked Desired button checked state.
   */
  void setExpandButtonChecked(bool checked);

  /**
   * @brief Refreshes channel combo contents in the settings widget.
   */
  void refreshSettingsChannels();

  /**
   * @brief Returns the settings scroll host for reparenting into the inspector.
   *
   * @return Settings container widget (non-owning for the caller).
   * @see recallSettingsWidget()
   */
  QWidget* settingsWidgetForInspector();

  /**
   * @brief Reclaims the settings widget from the shared inspector sidebar.
   */
  void recallSettingsWidget();

  /**
   * @brief Drain pending image messages and repaint.
   *
   * Safe to call when the panel is unfocused (e.g. global playback tick).
   */
  void tick();

  /**
   * @brief Push a decoded frame from Image Display (UI thread).
   *
   * @param image Frame produced by an Image display plugin.
   */
  void setFrameFromDisplay(const QImage& image);

  /**
   * @brief Exports the current display frame as PNG via a file dialog.
   *
   * @param with_annotations When @c true, burns annotation layers into the
   *        export; otherwise saves the composite image only.
   */
  void exportImageAsPng(bool with_annotations = false);

 signals:
  /**
   * @brief Emitted when panel config changes and should be persisted.
   */
  void configChanged();

  /**
   * @brief Emitted when this panel becomes the active panel (focus / click).
   */
  void activated();

  /**
   * @brief Emitted when settings visibility is toggled from the title bar.
   *
   * @param visible New settings visibility.
   */
  void settingsToggled(bool visible);

  /**
   * @brief Request splitting this panel in the given orientation.
   *
   * @param orientation Horizontal or vertical split.
   */
  void panelSplitRequested(Qt::Orientation orientation);

  /**
   * @brief Request removing this panel from the layout.
   */
  void panelRemoveRequested();

  /**
   * @brief Request expanding / maximizing this panel.
   */
  void panelExpandRequested();

  /**
   * @brief Request changing the panel type by object name.
   *
   * @param object_name Target panel type object name.
   */
  void panelChangeRequested(const QString& object_name);

 protected:
  /**
   * @brief Emits @ref activated() when the panel receives focus.
   *
   * @param event Focus-in event.
   */
  void focusInEvent(QFocusEvent* event) override;

  /**
   * @brief Accepts channel / series drag payloads for image channels.
   *
   * @param event Drag-enter event.
   */
  void dragEnterEvent(QDragEnterEvent* event) override;

  /**
   * @brief Applies a dropped channel as the main image source.
   *
   * @param event Drop event carrying channel mime data.
   */
  void dropEvent(QDropEvent* event) override;

 private slots:
  /**
   * @brief Title-bar Settings toggle handler.
   *
   * @param visible Requested settings visibility.
   */
  void onToggleSettings(bool visible);

  /**
   * @brief Frame timer: drains queues and updates the rendered frame.
   */
  void onFrameTick();

 private:
  /**
   * @struct OverlayRuntime
   * @brief Per-overlay subscription, queue, and latest decoded image.
   */
  struct OverlayRuntime {
    ImageOverlayConfig config;  /**< Overlay style and channel. */
    integration::ChannelReaderRegistry::SubscriptionId subscription_id = 0;  /**< Active sub id. */
    integration::MessageQueue queue;  /**< Incoming overlay payloads. */
    QImage image;                   /**< Latest decoded overlay frame. */
    qint64 timestamp_ns = 0;        /**< Latest overlay timestamp. */
  };

  /**
   * @struct AnnotationRuntime
   * @brief Per-annotation-channel subscription and parsed layer.
   */
  struct AnnotationRuntime {
    QString channel;  /**< Annotation channel name. */
    integration::ChannelReaderRegistry::SubscriptionId subscription_id = 0;  /**< Active sub id. */
    integration::MessageQueue queue;  /**< Incoming annotation payloads. */
    ImageAnnotationLayer layer;     /**< Latest parsed annotation layer. */
    qint64 timestamp_ns = 0;        /**< Latest annotation timestamp. */
    std::string message_type;       /**< Last payload message type. */
    std::string last_payload;       /**< Last raw payload (for Detection stats). */
  };

  /**
   * @struct MarkerRuntime
   * @brief Per-marker-channel subscription and projected annotation layer.
   */
  struct MarkerRuntime {
    QString channel;  /**< Marker channel name. */
    integration::ChannelReaderRegistry::SubscriptionId subscription_id = 0;  /**< Active sub id. */
    integration::MessageQueue queue;  /**< Incoming marker payloads. */
    ImageAnnotationLayer layer;     /**< Latest projected marker layer. */
    qint64 timestamp_ns = 0;        /**< Latest marker timestamp. */
  };

  /**
   * @struct PointCloudRuntime
   * @brief Per-PointCloud2 subscription and projected annotation layer.
   */
  struct PointCloudRuntime {
    QString channel;
    integration::ChannelReaderRegistry::SubscriptionId subscription_id = 0;
    integration::MessageQueue queue;
    ImageAnnotationLayer layer;
    qint64 timestamp_ns = 0;
  };

  /** Tear down and rebuild all subscriptions from @c config_. */
  void resubscribeAll();

  /** Unsubscribe the main image channel only. */
  void unsubscribeMain();

  /** Subscribe the main image channel from @c config_.image_channel. */
  void subscribeMain();

  /** Unsubscribe all overlay channels. */
  void unsubscribeOverlays();

  /** Subscribe enabled overlay channels. */
  void subscribeOverlays();

  /** Unsubscribe all annotation channels. */
  void unsubscribeAnnotations();

  /** Subscribe annotation channels from config. */
  void subscribeAnnotations();

  /** Unsubscribe CameraInfo / calibration channel. */
  void unsubscribeCalibration();

  /** Subscribe calibration channel when configured. */
  void subscribeCalibration();

  /** Unsubscribe all marker channels. */
  void unsubscribeMarkers();

  /** Subscribe marker channels from config. */
  void subscribeMarkers();

  /** Unsubscribe all PointCloud2 projection channels. */
  void unsubscribePointClouds();

  /** Subscribe PointCloud2 channels from config. */
  void subscribePointClouds();

  /** Drain main / overlay / annotation / calibration / marker queues. */
  void drainIncomingQueues();

  /**
   * @brief Handle a main-image payload (decode + store @c base_image_).
   *
   * @param payload Serialized image / compressed / video packet.
   */
  void handleMainPayload(const std::string& payload);

  /**
   * @brief Handle an overlay payload at @p index.
   *
   * @param index Index into @c overlay_runtime_.
   * @param payload Serialized overlay image.
   */
  void handleOverlayPayload(int index, const std::string& payload);

  /**
   * @brief Handle an annotation payload at @p index.
   *
   * @param index Index into @c annotation_runtime_.
   * @param payload Serialized annotation message.
   */
  void handleAnnotationPayload(int index, const std::string& payload);

  /**
   * @brief Handle a CameraInfo payload and refresh intrinsics / TF.
   *
   * @param payload Serialized CameraInfo.
   */
  void handleCalibrationPayload(const std::string& payload);

  /**
   * @brief Handle a marker payload at @p index and project to pixels.
   *
   * @param index Index into @c marker_runtime_.
   * @param payload Serialized Marker / MarkerArray.
   */
  void handleMarkerPayload(int index, const std::string& payload);

  /**
   * @brief Handle a PointCloud2 payload at @p index and project to pixels.
   *
   * @param index Index into @c point_cloud_runtime_.
   * @param payload Serialized PointCloud2.
   */
  void handlePointCloudPayload(int index, const std::string& payload);

  /**
   * @brief Decode a payload into an RGB @c QImage (raw, compressed, or video).
   *
   * @param message_type Schema type name for the channel.
   * @param payload Serialized bytes.
   * @return Decoded image, or a null image on failure.
   */
  QImage decodePayload(const std::string& message_type,
                       const std::string& payload);

  /**
   * @brief Looks up the message type string for a channel via the manager.
   *
   * @param channel Channel name.
   * @return Message type, or empty if unknown.
   */
  std::string messageTypeForChannel(const std::string& channel) const;

  /** Recompute @c fixed_to_optical_ from CameraInfo + TF. */
  void updateFixedToOptical();

  /**
   * @brief Appends primitives from @p source into @p destination.
   *
   * @param destination Layer to mutate.
   * @param source Layer to merge in.
   */
  void mergeAnnotationLayer(ImageAnnotationLayer* destination,
                            const ImageAnnotationLayer& source) const;

  /** Composite base + overlays + annotations and push to the view. */
  void updateRenderedFrame();

  /** Refresh view HUD (channel, resolution, FPS, calib / undistort). */
  void updateViewHud();

  /** Record a frame arrival for the FPS sliding window. */
  void noteFrameArrival();

  /** Rebuild analysis toolbar / histogram from the current display frame. */
  void refreshAnalysisPanel();

  /** Apply tool mode from toolbar buttons. */
  void setActiveTool(ImageViewTool tool);

  /** Show / hide the floating bottom analysis chrome. */
  void setFloatingChromeVisible(bool visible);

  /** Sync histogram / profile strip visibility with tool state. */
  void updateAnalysisStripVisibility();

  /**
   * @brief Publish a pixel click/hover sample onto a channel.
   *
   * @param channel Destination channel name (empty = no-op).
   * @param x Image-pixel X.
   * @param y Image-pixel Y.
   */
  void publishPixel(const QString& channel, int x, int y) const;

  /** Push @c config_ into @c settings_widget_ without emitting loops. */
  void syncSettingsWidgetFromConfig();

  /** Sync title-bar tool button checked states from config / UI. */
  void syncSettingsToolState();

  /** Apply title, background, and view options from @c config_ to widgets. */
  void applyConfigToUi();

  /** Non-owning; channel discovery and subscriptions. */
  common::VisualizationManager* manager_ = nullptr;

  /** Persisted / edited panel configuration. */
  ImagePanelConfig config_;

  /** Main image view (pan / zoom / annotations). */
  ImageViewWidget* view_ = nullptr;

  /** Host stacking @c view_ and the floating bottom chrome. */
  QWidget* image_host_ = nullptr;

  /** Bottom inset wrapper (margins) for the floating chrome. */
  QWidget* overlay_wrap_ = nullptr;

  /** Frosted glass card holding tools + analysis widgets. */
  QWidget* overlay_chrome_ = nullptr;

  /** Compact tool strip: Probe / Measure / ROI / Histogram. */
  QWidget* tool_bar_ = nullptr;

  /** Exclusive tool buttons. */
  QButtonGroup* tool_group_ = nullptr;

  /** Histogram + stats strip under the tool row (inside chrome). */
  QWidget* analysis_strip_ = nullptr;

  /** One-click hide for the floating chrome. */
  QToolButton* overlay_hide_button_ = nullptr;

  /** Small pill to restore the floating chrome when hidden. */
  QToolButton* overlay_reveal_button_ = nullptr;

  /** Luminance histogram with colormap range handles. */
  ImageHistogramWidget* histogram_widget_ = nullptr;

  /** Line profile chart under the histogram. */
  ImageProfileWidget* profile_widget_ = nullptr;

  /** Measure plane depth (meters) for metric distance. */
  QDoubleSpinBox* measure_depth_spin_ = nullptr;

  /** "Depth" label beside @c measure_depth_spin_ (Measure tool only). */
  QLabel* measure_depth_label_ = nullptr;

  /** ROI / measure textual summary. */
  QLabel* analysis_label_ = nullptr;

  /** Whether the histogram strip is visible. */
  bool histogram_visible_ = false;

  /** Whether the floating bottom chrome is expanded. */
  bool floating_chrome_visible_ = false;

  /** Property editor for channels and display options. */
  ImageSettingsWidget* settings_widget_ = nullptr;

  /** Scroll host wrapping settings (inspector reparent target). */
  QScrollArea* settings_scroll_ = nullptr;

  /** Container holding settings scroll inside the panel layout. */
  QWidget* settings_container_ = nullptr;

  /** Dock title-bar Settings button (weak). */
  QPointer<QToolButton> settings_button_;

  /** Dock title-bar Expand button (weak). */
  QPointer<QToolButton> expand_button_;

  /** Main image subscription id (@c 0 = none). */
  integration::ChannelReaderRegistry::SubscriptionId main_subscription_id_ = 0;

  /** Calibration subscription id (@c 0 = none). */
  integration::ChannelReaderRegistry::SubscriptionId calibration_subscription_id_ = 0;

  /** Incoming main-image message queue. */
  integration::MessageQueue main_queue_;

  /** Incoming CameraInfo message queue. */
  integration::MessageQueue calibration_queue_;

  /** Latest decoded main image (pre-composite). */
  QImage base_image_;

  /** Timestamp of @c base_image_ (nanoseconds). */
  qint64 base_timestamp_ns_ = 0;

  /** Parsed camera intrinsics from calibration. */
  CameraIntrinsics camera_intrinsics_;

  /** Last CameraInfo message (for optical-frame math). */
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;

  /** Whether @c camera_info_ / intrinsics are usable. */
  bool have_camera_info_ = false;

  /** Fixed → optical transform for marker projection. */
  QMatrix4x4 fixed_to_optical_;

  /** Whether @c fixed_to_optical_ is valid. */
  bool have_fixed_to_optical_ = false;

  /** Per-overlay runtime state. */
  std::vector<OverlayRuntime> overlay_runtime_;

  /** Per-annotation-channel runtime state. */
  std::vector<AnnotationRuntime> annotation_runtime_;

  /** Per-marker-channel runtime state. */
  std::vector<MarkerRuntime> marker_runtime_;

  /** Per-PointCloud2-channel runtime state. */
  std::vector<PointCloudRuntime> point_cloud_runtime_;

  /** Stateful H.264/H.265/VP9 decoder for compressed video topics. */
  VideoStreamDecoder video_decoder_;

  /** Periodic drain / repaint timer. */
  QTimer* frame_timer_ = nullptr;

  /** Sliding window of recent frame arrival times for FPS HUD. */
  QVector<qint64> frame_arrival_ms_;

  /** Last computed display FPS (0 = unknown). */
  double display_fps_ = 0.0;
};

}  // namespace image
}  // namespace autoviz
