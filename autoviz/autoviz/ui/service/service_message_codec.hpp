/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_message_codec.hpp
 * @brief JSON ↔ protobuf encode/decode helpers for Service Call requests and
 *        responses.
 *
 * Parallel to @ref publish_panel::PublishMessageCodec but scoped to service
 * message types used by @ref ServiceEditorWidget.
 *
 * @see ServiceEditorWidget
 * @see publish_panel::PublishMessageCodec
 */

#pragma once

#include <optional>
#include <string>

#include <QString>

namespace autoviz {
namespace service_panel {

/**
 * @struct CodecResult
 * @brief Outcome of an encode or decode operation.
 *
 * On failure @c ok is @c false and @c error describes the problem; on success
 * @c payload (binary) and/or @c text (JSON) are populated depending on the call.
 */
struct CodecResult {
  bool ok = false;      /**< @c true when the operation succeeded. */
  QString error;        /**< Human-readable error when @c ok is @c false. */
  std::string payload;  /**< Serialized protobuf bytes (encode path). */
  QString text;         /**< JSON text (decode path / diagnostics). */
};

/**
 * @class ServiceMessageCodec
 * @brief Process-wide singleton for service request/response JSON codecs.
 *
 * Provides default JSON templates and encode/decode against fully-qualified
 * protobuf type names. Obtain via @ref instance().
 *
 * @note Does not perform the RPC itself; @ref ServiceEditorWidget owns the
 *       call lifecycle.
 *
 * @see CodecResult
 */
class ServiceMessageCodec {
 public:
  /**
   * @brief Returns the process-wide codec singleton.
   *
   * @return Reference to the shared @c ServiceMessageCodec.
   */
  static ServiceMessageCodec& instance();

  /**
   * @brief Builds a default empty JSON template for @p message_type.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @return JSON string if the type is known; @c std::nullopt otherwise.
   */
  std::optional<QString> defaultJsonTemplate(const std::string& message_type) const;

  /**
   * @brief Encodes editor JSON into a binary protobuf request payload.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @param message_json Request body as JSON text.
   * @return @ref CodecResult with @c payload set on success.
   */
  CodecResult encodeJson(const std::string& message_type,
                         const QString& message_json) const;

  /**
   * @brief Decodes a binary response payload into JSON for the editor.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @param bytes Serialized protobuf bytes.
   * @return @ref CodecResult with @c text set on success.
   */
  CodecResult decodeToJson(const std::string& message_type,
                           const std::string& bytes) const;

 private:
  /** @brief Private constructor; use @ref instance(). */
  ServiceMessageCodec() = default;
};

}  // namespace service_panel
}  // namespace autoviz
