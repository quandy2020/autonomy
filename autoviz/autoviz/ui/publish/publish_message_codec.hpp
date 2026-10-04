/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_message_codec.hpp
 * @brief Protobuf create / JSON encode-decode helpers for the Publish panel.
 *
 * Wraps DynamicFactory / generated message pools so the UI can list types,
 * fill default JSON templates, and turn editor JSON into wire payloads (and
 * the reverse for "fill from latest").
 *
 * @see PublishEditorWidget
 * @see ServiceMessageCodec
 */

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QString>

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace publish_panel {

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
  QString text;         /**< JSON text (decode path / diagnostics). */
  std::string payload;  /**< Serialized protobuf bytes (encode path). */
};

/**
 * @brief Creates a protobuf message instance for @p message_type.
 *
 * Tries the generated descriptor pool first, then falls back to
 * DynamicFactory for types discovered at runtime.
 *
 * @param message_type Fully-qualified protobuf type name.
 * @return Owned message prototype, or @c nullptr if the type is unknown.
 *
 * @see PublishMessageCodec::defaultJsonTemplate()
 */
std::unique_ptr<google::protobuf::Message> CreatePublishMessage(
    const std::string& message_type);

/**
 * @class PublishMessageCodec
 * @brief Process-wide singleton for listing types and JSON ↔ protobuf codecs.
 *
 * Thread-affinity follows Qt UI usage (main thread). Callers obtain the shared
 * instance via @ref instance().
 *
 * @note Does not own channel writers; encoding only produces bytes / JSON.
 *
 * @see CreatePublishMessage()
 * @see CodecResult
 */
class PublishMessageCodec {
 public:
  /**
   * @brief Returns the process-wide codec singleton.
   *
   * @return Reference to the shared @c PublishMessageCodec.
   */
  static PublishMessageCodec& instance();

  /**
   * @brief Lists known publishable message type names.
   *
   * @return Sorted (or discovery-order) fully-qualified type strings.
   */
  std::vector<std::string> listMessageTypes() const;

  /**
   * @brief Builds a default JSON template for an empty message of @p message_type.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @return JSON string if the type is known; @c std::nullopt otherwise.
   */
  std::optional<QString> defaultJsonTemplate(const std::string& message_type) const;

  /**
   * @brief Encodes editor JSON into a binary protobuf payload.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @param message_json Message body as JSON text.
   * @return @ref CodecResult with @c payload set on success.
   */
  CodecResult encodeJson(const std::string& message_type,
                         const QString& message_json) const;

  /**
   * @brief Decodes a binary payload into pretty JSON for the editor.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @param bytes Serialized protobuf bytes.
   * @return @ref CodecResult with @c text set on success.
   */
  CodecResult decodePayloadToJson(const std::string& message_type,
                                  const std::string& bytes) const;

 private:
  /** @brief Private constructor; use @ref instance(). */
  PublishMessageCodec() = default;
};

}  // namespace publish_panel
}  // namespace autoviz
