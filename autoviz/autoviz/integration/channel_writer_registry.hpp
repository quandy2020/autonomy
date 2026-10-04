/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_writer_registry.hpp
 * @brief Process-wide cache of one Autolink RawMessage writer per channel.
 *
 * Matches teleop UI and @c autolink echo publish paths: reuse writers, wait
 * optionally for a reader, and expose @ref PublishDiagnostics for the UI.
 *
 * @see ChannelReaderRegistry
 * @see PublishDiagnostics
 * @see teleop_channels.hpp
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"

namespace autoviz {
namespace integration {

/**
 * @struct PublishDiagnostics
 * @brief Last publish attempt diagnostics for UI status / troubleshooting.
 */
struct PublishDiagnostics {
  /** Topology reports at least one reader on the channel. */
  bool topology_has_reader = false;
  /** The cached writer currently has a connected reader. */
  bool writer_has_reader = false;
  /** Human-readable detail (timeout, schema mismatch, success note, …). */
  std::string detail;
};

/**
 * @class ChannelWriterRegistry
 * @brief Singleton caching RawMessage writers keyed by channel name.
 *
 * ## Publish modes
 *
 * - @ref publish(): may wait briefly for a reader (interactive / one-shot).
 * - @ref publishLoop(): non-blocking single write (high-rate teleop loops).
 *
 * @note Thread-safe. Call @ref setNode() after Autolink initialization.
 */
class ChannelWriterRegistry {
 public:
  /**
   * @brief Returns the process-wide registry singleton.
   * @return Reference to the shared instance.
   */
  static ChannelWriterRegistry& instance();

  /**
   * @brief Attaches the Autolink node used to create writers.
   *
   * @param node Shared node from @ref AutolinkContext; empty clears the weak
   *        handle.
   */
  void setNode(const std::shared_ptr<::autolink::Node>& node);

  /**
   * @brief Publishes @p payload on @p channel, waiting for a reader when needed.
   *
   * @param channel Fully-qualified Autolink channel name.
   * @param payload Serialized message bytes.
   * @param message_type Optional schema / type hint for writer creation.
   * @return @c true if the write succeeded.
   * @see lastDiagnostics()
   */
  bool publish(const std::string& channel, const std::string& payload,
               const std::string& message_type = {});

  /**
   * @brief Non-blocking loop publish: no reader wait, single write attempt.
   *
   * Intended for continuous teleop / echo-style loops where blocking would
   * stall the caller.
   *
   * @param channel Fully-qualified Autolink channel name.
   * @param payload Serialized message bytes.
   * @param message_type Optional schema / type hint for writer creation.
   * @return @c true if the write succeeded.
   */
  bool publishLoop(const std::string& channel, const std::string& payload,
                   const std::string& message_type = {});

  /**
   * @brief Returns diagnostics from the most recent publish attempt.
   * @return Copy of @c last_diagnostics_.
   */
  PublishDiagnostics lastDiagnostics() const;

 private:
  /** @brief Private default constructor (singleton). */
  ChannelWriterRegistry() = default;

  /**
   * @brief Ensures a writer exists for @p channel with optional type hint.
   *
   * @param channel Channel to bind.
   * @param message_type Schema type; may force recreation if it changes.
   * @return @c true if a usable writer is available.
   *
   * @note Caller must hold @c mutex_.
   */
  bool ensureWriterLocked(const std::string& channel,
                          const std::string& message_type);

  /**
   * @brief Drops the cached writer for @p channel.
   * @param channel Channel whose writer should be reset.
   *
   * @note Caller must hold @c mutex_.
   */
  void resetWriterLocked(const std::string& channel);

  /**
   * @brief Blocks until @p writer has a reader or @p timeout_ms elapses.
   *
   * @param writer Writer to poll.
   * @param channel Channel name (for diagnostics text).
   * @param timeout_ms Maximum wait in milliseconds.
   * @return @c true if a reader was observed before timeout.
   */
  bool waitForWriterReader(
      const std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>&
          writer,
      const std::string& channel, int timeout_ms) const;

  /**
   * @brief Updates @c last_diagnostics_ under @c diagnostics_mutex_.
   *
   * @param topology_has_reader Topology reader presence.
   * @param writer_has_reader Writer-connected reader presence.
   * @param detail Human-readable status string.
   */
  void setDiagnostics(bool topology_has_reader, bool writer_has_reader,
                      const std::string& detail);

  /** Guards @c node_, @c writers_, and @c writer_schema_types_. */
  std::mutex mutex_;

  /** Guards @c last_diagnostics_. */
  mutable std::mutex diagnostics_mutex_;

  /** Most recent publish diagnostics snapshot. */
  PublishDiagnostics last_diagnostics_;

  /** Weak Autolink node used to create writers. */
  std::weak_ptr<::autolink::Node> node_;

  /** Cached writers keyed by channel name. */
  std::unordered_map<
      std::string,
      std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>>
      writers_;

  /** Schema / message type used when each writer was created. */
  std::unordered_map<std::string, std::string> writer_schema_types_;
};

}  // namespace integration
}  // namespace autoviz
