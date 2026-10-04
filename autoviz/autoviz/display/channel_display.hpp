/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_display.hpp
 * @brief Templated @ref Display that subscribes to one Autolink channel and
 *        parses protobuf payloads of type @c ProtoT.
 *
 * Most topic-backed Autoviz displays inherit @ref ChannelDisplay rather than
 * wiring @ref integration::ChannelReaderRegistry themselves. The template:
 * - subscribes on @ref onEnable / unsubscribes on @ref onDisable;
 * - drains only the newest queued payload on @ref onUpdate (UI-thread safety);
 * - updates status for missing topic / parse failure / no messages;
 * - delegates typed handling to @ref processMessage.
 *
 * @tparam ProtoT Protobuf message type (e.g.
 *         @c automsgs::msgs::sensor_msgs::LaserScan).
 *
 * @see Display
 * @see integration::ChannelReaderRegistry
 * @see integration::MessageQueue
 */

#pragma once

#include <string>

#include "autoviz/display/display.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {
namespace {

/**
 * @brief Returns whether @p channel is a usable Autolink topic name.
 *
 * @param channel Candidate channel string.
 * @return @c true when non-empty and not a sentinel starting with @c '-'.
 */
bool IsValidChannelName(const std::string& channel) {
  return !channel.empty() && channel.front() != '-';
}

}  // namespace

/**
 * @class ChannelDisplay
 * @brief Display specialization that owns a single-channel subscription and
 *        typed protobuf decode path.
 *
 * ## Data flow
 *
 * - **In:** Autolink callback → @c queue_.push → @ref onUpdate takes latest →
 *   decode → @ref processMessage.
 * - **Out:** subclass @ref onDraw reads cached state built in
 *   @ref processMessage.
 *
 * @tparam ProtoT Parsed protobuf message type.
 *
 * @note @ref onDisable must not call @c request_redraw: destruction during
 *       VisualizationManager teardown would re-enter @c update() on destroyed
 *       displays (SIGSEGV).
 *
 * @see Display
 * @see MarkerDisplay
 * @see LaserScanDisplay
 */
template <typename ProtoT>
class ChannelDisplay : public Display {
 public:
  /**
   * @brief Constructs a channel-backed display with fixed type metadata.
   *
   * @param type_id Catalog type id returned by @ref typeId() (e.g.
   *        @c "LaserScan").
   * @param channel Initial Autolink channel; may be empty until the user
   *        picks a topic.
   * @param message_type Expected wire / schema type string (informational;
   *        parse uses @c ProtoT directly).
   */
  ChannelDisplay(std::string type_id, std::string channel,
                 std::string message_type)
      : type_id_(std::move(type_id)),
        channel_(std::move(channel)),
        message_type_(std::move(message_type)) {}

  /**
   * @brief Ensures unsubscribe via @ref onDisable before destruction.
   */
  ~ChannelDisplay() override { onDisable(); }

  /**
   * @brief Clears queue, receive flags, and subclass cached data.
   *
   * Calls @ref Display::reset, @ref clearReceivedData, and requests redraw
   * when a context is available.
   */
  void reset() override {
    Display::reset();
    queue_.clear();
    has_received_message_ = false;
    parse_failed_ = false;
    clearReceivedData();
    if (context_ != nullptr && context_->request_redraw) {
      context_->request_redraw();
    }
  }

  /**
   * @brief Returns the catalog type id passed to the constructor.
   *
   * @return @c type_id_.
   */
  std::string typeId() const override { return type_id_; }

  /**
   * @brief Returns the current Autolink channel name.
   *
   * @return @c channel_ (may be empty).
   */
  std::string channel() const override { return channel_; }

  /**
   * @brief Changes the subscribed channel, rebinding while enabled.
   *
   * Invalid names (empty or leading @c '-') are stored as empty and yield
   * @c "No topic set" / @c "Invalid channel name" status.
   *
   * @param channel New channel name.
   */
  void setChannel(const std::string& channel) override {
    if (channel_ == channel) {
      return;
    }
    const bool active = enabled();
    if (active) {
      onDisable();
    }
    channel_ = IsValidChannelName(channel) ? channel : std::string{};
    if (active) {
      onEnable();
    }
  }

 protected:
  /**
   * @brief Drops cached visuals derived from channel messages.
   *
   * Called from @ref reset (Time panel Reset). Default is a no-op; subclasses
   * clear point lists, images, marker maps, etc.
   */
  virtual void clearReceivedData() {}

  /**
   * @brief Subscribes when Autolink and channel are valid; else sets error
   *        status.
   */
  void onEnable() override {
    has_received_message_ = false;
    parse_failed_ = false;
    if (context_ == nullptr || context_->autolink == nullptr ||
        context_->autolink->node() == nullptr) {
      setStatusError("Autolink not ready");
      return;
    }
    if (channel_.empty()) {
      setStatusError("No topic set");
      return;
    }
    if (!IsValidChannelName(channel_)) {
      setStatusError("Invalid channel name");
      return;
    }
    subscribeChannel();
  }

  /**
   * @brief Unsubscribes and clears the payload queue without requesting
   *        redraw.
   *
   * @note Intentionally skips @c request_redraw during manager teardown.
   */
  void onDisable() override {
    if (subscription_id_ != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
      subscription_id_ = 0;
    }
    queue_.clear();
    // Do not request_redraw here: ~ChannelDisplay calls onDisable while
    // VisualizationManager is clearing displays_; a sync redraw would
    // manager_->update() into destroyed Display objects (SIGSEGV).
  }

  /**
   * @brief Re-subscribes if needed, takes the latest payload, parses
   *        @c ProtoT, and updates status.
   *
   * Only the newest sample is processed so a full backlog cannot freeze the
   * UI thread.
   */
  void onUpdate() override {
    if (subscription_id_ == 0 && !channel_.empty() && context_ != nullptr &&
        context_->autolink != nullptr && context_->autolink->node() != nullptr) {
      subscribeChannel();
    }
    // Visualization only needs the newest sample; draining/parsing a full
    // backlog on the UI thread freezes checkbox toggles and the 3D view.
    if (auto payload = queue_.takeLatest()) {
      ProtoT proto;
      const std::string decoded = integration::DecodeChannelPayload(*payload);
      if (proto.ParseFromString(decoded) || proto.ParseFromString(*payload)) {
        has_received_message_ = true;
        parse_failed_ = false;
        processMessage(proto);
      } else {
        parse_failed_ = true;
      }
    }
    if (channel_.empty()) {
      setStatusError("No topic set");
    } else if (parse_failed_) {
      setStatusWarn("Failed to parse message payload");
    } else if (!has_received_message_) {
      setStatusWarn("No messages received");
    }
  }

  /**
   * @brief Handles a successfully parsed channel message.
   *
   * @param message Typed protobuf instance for this frame's latest sample.
   */
  virtual void processMessage(const ProtoT& message) = 0;

  /**
   * @brief Registers a ChannelReaderRegistry subscription that pushes raw
   *        payloads into @c queue_.
   *
   * Sets error status when subscribe returns id @c 0.
   */
  void subscribeChannel() {
    if (subscription_id_ != 0 || channel_.empty()) {
      return;
    }
    subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
        channel_, [this](const std::string& payload) { queue_.push(payload); });
    if (subscription_id_ == 0) {
      setStatusError("Failed to subscribe channel");
    }
  }

 private:
  /** Catalog / factory type id. */
  std::string type_id_;

  /** Current Autolink channel (topic). */
  std::string channel_;

  /** Declared message type string (schema hint). */
  std::string message_type_;

  /** Thread-safe raw payload queue; only latest is taken in @ref onUpdate. */
  integration::MessageQueue queue_;

  /** Registry subscription handle; @c 0 means unsubscribed. */
  integration::ChannelReaderRegistry::SubscriptionId subscription_id_ = 0;

  /** Set after at least one successful parse since enable/reset. */
  bool has_received_message_ = false;

  /** Set when the latest payload failed protobuf parse. */
  bool parse_failed_ = false;
};

}  // namespace display
}  // namespace autoviz
