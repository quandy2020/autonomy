/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_types.hpp
 * @brief Shared configuration types for the Service Call panel.
 *
 * Defines @ref ServiceCallPanelConfig used by @ref ServicePanel,
 * @ref ServiceEditorWidget, and @ref ServiceSettingsWidget.
 *
 * @see ServicePanel
 * @see DefaultServiceCallPanelConfig()
 */

#pragma once

#include <QColor>
#include <QString>

namespace autoviz {
namespace service_panel {

/**
 * @struct ServiceCallPanelConfig
 * @brief Runtime configuration for one Service Call panel instance.
 *
 * Captures the selected service, request/response types and JSON bodies,
 * layout orientation, call timeout, and Call-button chrome.
 *
 * @see DefaultServiceCallPanelConfig()
 * @see ServicePanel::config()
 */
struct ServiceCallPanelConfig {
  QString title;              /**< Panel / dock title string. */
  QString service_name;       /**< Fully-qualified service channel / name. */
  QString request_type;       /**< Protobuf type of the request message. */
  QString response_type;      /**< Protobuf type of the response message. */
  QString request_json = QStringLiteral("{}"); /**< Request body as JSON. */
  QString response_json;      /**< Last response body as JSON (may be empty). */
  bool editing_mode = false;  /**< When @c true, show advanced type / layout UI. */
  bool vertical_layout = true; /**< Request above response when @c true. */
  int timeout_sec = 5;        /**< RPC timeout in seconds. */
  QString button_label = QStringLiteral("Call"); /**< Primary action label. */
  QString button_tooltip;     /**< Primary action tooltip (may be empty). */
  QColor button_color;        /**< Primary action accent; invalid = theme default. */
};

/**
 * @brief Returns a default @ref ServiceCallPanelConfig for a new panel.
 *
 * @return Sensible starting config (empty service, default timeout / button).
 */
ServiceCallPanelConfig DefaultServiceCallPanelConfig();

}  // namespace service_panel
}  // namespace autoviz
