/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_catalog.hpp
 * @brief Static catalog of built-in display types and message-type matching.
 *
 * Powers the Add Display dialog: lists available types, descriptions, and
 * which message types each display can subscribe to. Also special-cases
 * static TF channels (ingested by the TF listener, not as separate displays).
 *
 * @see DisplayRegistry
 * @see DisplayFactory
 * @see DisplayTypeInfo
 */

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace common {

/**
 * @struct DisplayTypeInfo
 * @brief Metadata for one display type shown in the Add Display UI.
 */
struct DisplayTypeInfo {
  /** Type id used in @ref DisplayConfig::type (e.g. @c "PointCloud2"). */
  std::string type;

  /** Package / plugin package name (informational). */
  std::string package;

  /** Human-readable description for the dialog. */
  std::string description;

  /**
   * Message type strings this display can consume.
   * Empty if the display does not subscribe to a channel (e.g. Grid, Axes).
   */
  std::vector<std::string> message_types;
};

/**
 * @class DisplayCatalog
 * @brief Query helpers over the built-in display type table.
 *
 * All methods are static; there is no mutable instance state.
 */
class DisplayCatalog {
 public:
  /**
   * @brief Returns metadata for every known display type.
   * @return Full catalog list.
   */
  static std::vector<DisplayTypeInfo> allTypes();

  /**
   * @brief Looks up catalog info for a single type id.
   *
   * @param type Display type string.
   * @return Matching @ref DisplayTypeInfo, or a default-constructed value
   *         if unknown.
   */
  static DisplayTypeInfo infoForType(const std::string& type);

  /**
   * @brief Lists display type ids that accept @p message_type.
   *
   * @param message_type Protobuf / Autolink message type name.
   * @return Matching display type ids (may be empty).
   */
  static std::vector<std::string> typesForMessageType(
      const std::string& message_type);

  /**
   * @brief Returns @c true for ROS-style static TF channels.
   *
   * Recognizes names such as @c /tf_static and @c /static_tf. These are
   * ingested by @c transform::Listener into the shared buffer and are not
   * separate TF Display sources (RViz2 also exposes only one TF display).
   *
   * @param channel Channel name to test.
   * @return Whether the channel is a static TF source.
   */
  static bool isStaticTfChannel(const std::string& channel);

  /**
   * @brief Display types offered for a live channel in Add Display.
   *
   * Skips TF display options when @p channel_name is a static TF channel.
   *
   * @param channel_name Live channel name.
   * @param message_type Message type advertised by the channel.
   * @return Ordered list of display type ids suitable for that channel.
   */
  static std::vector<std::string> typesForChannel(
      const std::string& channel_name, const std::string& message_type);
};

}  // namespace common
}  // namespace autoviz
