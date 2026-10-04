/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_editor_widget.hpp
 * @brief Main Service Call editor — pick service, edit request, show response.
 *
 * Reuses @ref publish_panel::PublishFieldTreeWidget for request/response field
 * trees and @ref ServiceMessageCodec for JSON ↔ protobuf. Owned by
 * @ref ServicePanel.
 *
 * ## Data flow
 *
 * - **Out:** edits emit @ref configChanged(); successful/failed calls emit
 *   @ref callFinished() after updating response JSON / status.
 * - **In:** @ref setConfig() restores UI; @ref refreshServices() refreshes
 *   the discovered service list.
 *
 * @see ServiceMessageCodec
 * @see ServicePanel
 * @see publish_panel::PublishFieldTreeWidget
 */

#pragma once

#include <QWidget>

#include <atomic>

class QKeyEvent;

#include "autoviz/ui/service/service_types.hpp"

class QCheckBox;
class QComboBox;
class QGroupBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSplitter;
class QTimer;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace publish_panel {
class PublishFieldTreeWidget;
}

namespace service_panel {

/**
 * @class ServiceEditorWidget
 * @brief Interactive editor for composing service requests and viewing replies.
 *
 * ## Layout
 *
 * @code
 * ┌─ service / types / refresh ───────────────────────────────┐
 * │ Request (fields + JSON)  │  Response (fields + JSON)      │
 * │                          │  (orientation from settings)   │
 * ├───────────────────────────────────────────────────────────┤
 * │ status                          [ Call ]                  │
 * └───────────────────────────────────────────────────────────┘
 * @endcode
 *
 * Calls are asynchronous; @c call_in_progress_ guards re-entrancy while a
 * request is outstanding.
 *
 * @note Does not own @ref common::VisualizationManager.
 */
class ServiceEditorWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the editor UI and wires signals.
   *
   * @param manager Non-owning visualization manager for service discovery and
   *        RPC; may be @c nullptr until later (Call stays disabled).
   * @param parent Qt parent (typically @ref ServicePanel content).
   */
  explicit ServiceEditorWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  /**
   * @brief Returns the current service-call configuration.
   *
   * @return Copy of @c config_ synchronized from the UI.
   * @see setConfig()
   */
  ServiceCallPanelConfig config() const;

  /**
   * @brief Replaces configuration and refreshes all editor widgets.
   *
   * @param config Full @ref ServiceCallPanelConfig to apply.
   * @see config()
   */
  void setConfig(const ServiceCallPanelConfig& config);

  /**
   * @brief Rebuilds the service combo from discovery.
   *
   * Lightweight refresh when the service list changes without a full
   * @ref setConfig().
   */
  void refreshServices();

  /**
   * @brief Applies vertical or horizontal request/response splitter orientation.
   *
   * @param vertical When @c true, request above response; otherwise side-by-side.
   * @see ServiceCallPanelConfig::vertical_layout
   */
  void applyLayoutOrientation(bool vertical);

 signals:
  /**
   * @brief Emitted when durable editor state changes.
   *
   * Listeners should persist @ref config() into session config.
   */
  void configChanged();

  /**
   * @brief Emitted when an in-flight service call completes (success or error).
   *
   * Response JSON / status are already updated before emission.
   */
  void callFinished();

 protected:
  /**
   * @brief Keyboard shortcuts (e.g. trigger Call).
   *
   * @param event Key event from Qt.
   */
  void keyPressEvent(QKeyEvent* event) override;

 private slots:
  /**
   * @brief Editing-mode checkbox: shows/hides advanced type / layout chrome.
   *
   * @param enabled New checkbox state.
   */
  void onEditingModeToggled(bool enabled);

  /**
   * @brief Service combo changed: resolves request/response types.
   *
   * @param text Selected service name.
   */
  void onServiceChanged(const QString& text);

  /**
   * @brief Request-type combo changed: may fill a default JSON template.
   *
   * @param text Fully-qualified request type name.
   */
  void onRequestTypeChanged(const QString& text);

  /** @brief Reloads the default JSON template for the current request type. */
  void onResetTemplate();

  /** @brief Request JSON plain-text edited; marks fields dirty / syncs tree. */
  void onFieldEdited();

  /** @brief Request field-tree edited; syncs JSON and emits config. */
  void onRequestTreeEdited();

  /** @brief Call button: encodes request and starts the async RPC. */
  void onCallClicked();

  /** @brief Refresh button: rediscovers available services. */
  void onRefreshServices();

 private:
  /**
   * @brief Factory for styled JSON plain-text editors.
   *
   * @param parent Parent widget for the editor.
   * @param placeholder Placeholder text when empty.
   * @param read_only When @c true, editor is not user-editable.
   * @return Owned @c QPlainTextEdit (parented to @p parent).
   */
  QPlainTextEdit* makeJsonEditor(QWidget* parent, const QString& placeholder,
                                 bool read_only = false);

  /** @brief Shows/hides advanced chrome based on editing mode. */
  void applyEditingModeUi();

  /** @brief Applies Call-button label / color from @c config_. */
  void applyButtonStyle();

  /** @brief Enables/disables Call based on service / type / in-flight state. */
  void updateCallButtonState();

  /**
   * @brief Updates the status label (optionally as an error style).
   *
   * @param text Status line to show.
   * @param is_error When @c true, apply error styling.
   */
  void updateStatus(const QString& text, bool is_error = false);

  /**
   * @brief Fills a default request template unless the user already edited.
   *
   * @param message_type Request type whose template should be loaded.
   */
  void maybeFillTemplateForType(const QString& message_type);

  /**
   * @brief Looks up and applies request/response types for @p service_name.
   *
   * @param service_name Selected service channel / name.
   */
  void resolveTypesForService(const QString& service_name);

  /** @brief Writes the request field tree into @c request_edit_ / config. */
  void syncRequestJsonFromTree();

  /** @brief Loads the request field tree from request JSON. */
  void syncRequestTreeFromJson();

  /**
   * @brief Loads the (read-only) response field tree from @p json.
   *
   * @param json Response body as JSON.
   */
  void syncResponseTreeFromJson(const QString& json);

  /** @brief Sets the request tree root label to the current service name. */
  void applyRequestServiceRootLabel();

  /** @brief Emits @ref configChanged() unless suppressed. */
  void emitConfigChanged();

  /**
   * @brief Completes an async call on the UI thread: updates response / status.
   *
   * @param snapshot Config used for the call (types / service name).
   * @param ok Whether the RPC succeeded.
   * @param response_text Decoded response JSON (may be empty on failure).
   * @param error_text Human-readable error when @p ok is @c false.
   */
  void finishCall(const ServiceCallPanelConfig& snapshot, bool ok,
                  const QString& response_text, const QString& error_text);

  /** Non-owning; service discovery and RPC path. */
  common::VisualizationManager* manager_ = nullptr;

  /** Canonical editor configuration. */
  ServiceCallPanelConfig config_;

  /** Guard: ignore type-combo template side effects while rebuilding. */
  bool suppress_template_update_ = false;

  /** @c true when request field-tree changes are not yet synced to JSON. */
  bool request_fields_dirty_ = false;

  /** Atomic flag: a service call is currently in flight. */
  std::atomic<bool> call_in_progress_{false};

  QWidget* rqt_top_bar_ = nullptr;            /**< Service / type toolbar. */
  QCheckBox* editing_mode_check_ = nullptr;   /**< Show advanced editor. */
  QLabel* status_label_ = nullptr;            /**< Call status / errors. */
  QWidget* advanced_body_ = nullptr;          /**< Advanced type / layout body. */
  QComboBox* service_combo_ = nullptr;        /**< Discovered service names. */
  QComboBox* request_type_combo_ = nullptr;   /**< Request protobuf type. */
  QComboBox* response_type_combo_ = nullptr;  /**< Response protobuf type. */
  QPushButton* refresh_services_button_ = nullptr; /**< Rediscover services. */
  QPushButton* reset_template_button_ = nullptr;   /**< Reload request template. */
  QSplitter* payload_splitter_ = nullptr;     /**< Request vs response split. */
  QGroupBox* request_group_ = nullptr;        /**< Request payload group. */
  QGroupBox* response_group_ = nullptr;       /**< Response payload group. */
  publish_panel::PublishFieldTreeWidget* request_tree_ = nullptr;  /**< Request fields. */
  publish_panel::PublishFieldTreeWidget* response_tree_ = nullptr; /**< Response fields. */
  QPlainTextEdit* request_edit_ = nullptr;    /**< Request JSON editor. */
  QPlainTextEdit* response_edit_ = nullptr;   /**< Response JSON viewer. */
  QPushButton* call_button_ = nullptr;        /**< Primary Call action. */
  QTimer* service_timer_ = nullptr;           /**< Periodic service-list refresh. */
};

}  // namespace service_panel
}  // namespace autoviz
