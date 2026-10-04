/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_types.hpp
 * @brief Shared data types and helpers for the Publish panel (rqt-style message
 *        publisher).
 *
 * Defines the in-memory configuration model used by @ref PublishPanel,
 * @ref PublishEditorWidget, @ref PublishSettingsWidget, and
 * @ref PublishFieldTreeWidget, plus small utilities for entry IDs and JSON
 * expression previews.
 *
 * @see publish_config_io.hpp
 * @see PublishPanel
 * @see common::PublishPanelPersistConfig
 */

#pragma once

#include <QColor>
#include <QString>
#include <QStringList>
#include <QVector>

namespace autoviz {
namespace publish_panel {

/**
 * @struct PublishEntry
 * @brief One active publisher row in the rqt-style multi-publisher table.
 *
 * Corresponds to a top-level row in @ref PublishFieldTreeWidget when
 * @c DisplayMode::kRqtPublishers is active (channel | type | rate | expression).
 *
 * @see PublishPanelConfig::publishers
 * @see PublishFieldTreeWidget::setPublishers()
 */
struct PublishEntry {
  QString id;                 /**< Stable UUID for timer / selection tracking. */
  QString channel;            /**< Topic / channel name to publish on. */
  QString message_type;       /**< Fully-qualified protobuf type name. */
  double publish_rate_hz = 1.0; /**< Loop publish rate; unused when not looping. */
  QString message_json;       /**< Message body as JSON (expression column). */
  bool publishing = true;     /**< When @c true, timer may publish this entry. */
};

/**
 * @struct PublishPreset
 * @brief Named snapshot of channel / type / JSON that the user can recall.
 *
 * Stored in @ref PublishPanelConfig::saved_presets and selected via the
 * editor preset combo.
 *
 * @see PublishEditorWidget
 */
struct PublishPreset {
  QString name;               /**< Display name in the preset combo. */
  QString channel;            /**< Channel associated with this preset. */
  QString message_type;       /**< Protobuf type for the preset payload. */
  QString message_json;       /**< Preset message body as JSON. */
  bool loop_publish = false;  /**< Whether to enable periodic publish. */
  double publish_rate_hz = 1.0; /**< Rate when @c loop_publish is enabled. */
  QString button_label;       /**< Optional publish-button label override. */
  QString button_tooltip;     /**< Optional publish-button tooltip. */
  QColor button_color;        /**< Optional publish-button accent color. */
};

/**
 * @struct PublishPanelConfig
 * @brief Full runtime configuration for one Publish panel instance.
 *
 * Combines the single-message "draft" editor state, multi-publisher collection,
 * saved presets, and UI chrome (title / button appearance). Persisted via
 * @ref ToPersistConfig() / @ref FromPersistConfig().
 *
 * @see DefaultPublishPanelConfig()
 * @see PublishPanel::config()
 * @see common::PublishPanelPersistConfig
 */
struct PublishPanelConfig {
  QString title;              /**< Panel / dock title string. */
  QString channel;            /**< Draft / active channel name. */
  QString message_type;       /**< Draft protobuf message type. */
  QString message_json;       /**< Draft message body as JSON. */
  bool editing_mode = false;  /**< When @c true, show advanced edit UI. */
  bool loop_publish = false;  /**< Enable periodic publish for the draft. */
  double publish_rate_hz = 1.0; /**< Draft loop rate in Hz. */
  QString button_label = QStringLiteral("Publish"); /**< Primary action label. */
  QString button_tooltip;     /**< Primary action tooltip (may be empty). */
  QColor button_color;        /**< Primary action accent; invalid = theme default. */
  QVector<PublishPreset> saved_presets; /**< User-saved named presets. */
  QString active_preset_name; /**< Currently selected preset name, if any. */
  QStringList custom_channels; /**< Channels typed by the user (not from discovery). */
  QVector<PublishEntry> publishers; /**< Multi-publisher table rows. */
  int selected_publisher_index = -1; /**< Selected row in @c publishers (−1 = none). */
};

/**
 * @brief Allocates a new unique publisher entry id (UUID without braces).
 *
 * @return Non-empty id string suitable for @ref PublishEntry::id.
 */
QString NewPublishEntryId();

/**
 * @brief Builds a single-line truncated preview of a JSON expression.
 *
 * Collapses whitespace/newlines and truncates to ~96 characters with an
 * ellipsis, for display in the rqt publishers expression column.
 *
 * @param json Raw message JSON (may be multi-line).
 * @return Compact preview string.
 */
QString ExpressionPreview(const QString& json);

/**
 * @brief Returns a default @ref PublishPanelConfig (cmd_vel / TwistStamped).
 *
 * @return Sensible starting config for a newly created Publish panel.
 */
PublishPanelConfig DefaultPublishPanelConfig();

}  // namespace publish_panel
}  // namespace autoviz
