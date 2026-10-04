/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_manager.hpp
 * @brief Thin wrapper over Autolink topology ChannelManager for UI channel lists.
 *
 * Used by channel pickers and publish panels to enumerate discovered channels
 * and optionally filter to those that currently have a writer.
 *
 * @see TopologyGraph
 * @see ChannelWriterRegistry
 * @see AutolinkContext
 */

#pragma once

#include <string>
#include <vector>

#include "autolink/service_discovery/topology_manager.hpp"

namespace autoviz {
namespace integration {

/**
 * @struct ChannelInfo
 * @brief Snapshot of one Autolink channel as shown in Autoviz UI lists.
 */
struct ChannelInfo {
  /** Fully-qualified channel / topic name. */
  std::string channel_name;
  /** Protobuf / message type descriptor string (may be empty if unknown). */
  std::string message_type;
  /** @c true when topology reports at least one writer on this channel. */
  bool has_writer = false;
};

/**
 * @class ChannelManager
 * @brief Reads Autolink service-discovery channel topology for Autoviz panels.
 *
 * Non-owning wrapper around @c autolink::service_discovery::ChannelManagerPtr;
 * topology lifetime is managed by Autolink / @ref AutolinkContext.
 *
 * @note Does not subscribe or publish; listing only.
 */
class ChannelManager {
 public:
  /**
   * @brief Binds to an Autolink topology channel manager.
   *
   * @param channel_manager Shared pointer from Autolink service discovery;
   *        must remain valid for the lifetime of this wrapper.
   */
  explicit ChannelManager(
      ::autolink::service_discovery::ChannelManagerPtr channel_manager);

  /**
   * @brief Lists all channels currently known to topology.
   * @return Vector of @ref ChannelInfo (order depends on Autolink).
   */
  std::vector<ChannelInfo> listChannels() const;

  /**
   * @brief Lists channels that currently have at least one writer.
   * @return Subset of @ref listChannels() where @c has_writer is @c true.
   */
  std::vector<ChannelInfo> listWritableChannels() const;

 private:
  /** Non-owning Autolink topology channel manager. */
  ::autolink::service_discovery::ChannelManagerPtr channel_manager_;
};

}  // namespace integration
}  // namespace autoviz
