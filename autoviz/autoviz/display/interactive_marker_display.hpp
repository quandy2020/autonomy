/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file interactive_marker_display.hpp
 * @brief Subscribes to InteractiveMarker update/init topics and syncs marker
 *        state into a shared @ref InteractiveMarkerRegistry.
 *
 * RViz InteractiveMarkers display analogue. The display owns channel
 * subscription and drawing; the @ref InteractTool picks / drags via the
 * registry.
 *
 * @see InteractiveMarkerRegistry
 * @see drawInteractiveMarker
 * @see ChannelDisplay
 */

#pragma once

#include <map>
#include <memory>
#include <string>

#include <automsgs/msgs/visualization_msgs/interactive_marker_init.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_update.pb.h>
#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/display/interactive_marker_registry.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {

/**
 * @class InteractiveMarkerDisplay
 * @brief Channel display for visualization_msgs InteractiveMarker updates.
 *
 * Primary channel carries @c InteractiveMarkerUpdate. An optional init
 * reader/queue applies full snapshots. Local @c markers_ is pushed to
 * @c registry_ via @ref syncRegistry so the Interact tool sees the same
 * state.
 *
 * @note Call @ref setRegistry before enable so picks and feedback publish
 *       work; without a registry, drawing still works from local state.
 *
 * @see InteractiveMarkerRegistry
 * @see applyInteractiveMarkerUpdate
 * @see drawInteractiveMarker
 */
class InteractiveMarkerDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::
                                InteractiveMarkerUpdate> {
 public:
  /**
   * @brief Constructs the display bound to the update @p channel.
   *
   * @param channel Autolink channel for InteractiveMarkerUpdate.
   */
  explicit InteractiveMarkerDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "InteractiveMarkers".
   */
  std::string typeId() const override { return "InteractiveMarkers"; }

  /**
   * @brief Property schema (show controls, feedback channel overrides, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  /**
   * @brief Attaches the shared registry used by the Interact tool.
   *
   * @param registry Non-owning; typically owned by VisualizationManager /
   *        frame. May be @c nullptr to detach.
   */
  void setRegistry(InteractiveMarkerRegistry* registry) { registry_ = registry; }

 protected:
  /**
   * @brief Enables update subscription plus optional init reader.
   */
  void onEnable() override;

  /**
   * @brief Disables readers and removes this source from the registry.
   */
  void onDisable() override;

  /**
   * @brief Drains init queue then update payloads; syncs registry.
   */
  void onUpdate() override;

  /**
   * @brief Applies an InteractiveMarkerUpdate to @c markers_.
   *
   * @param message Update protobuf (KEEP_ALIVE / UPDATE / etc.).
   */
  void processMessage(
      const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
          message) override;

  /**
   * @brief Clears local markers and registry entries for this source.
   */
  void clearReceivedData() override;

  /**
   * @brief Draws each interactive marker via @ref drawInteractiveMarker.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @brief Pushes @c markers_ into @c registry_ under @ref sourceId().
   */
  void syncRegistry();

  /**
   * @brief Stable id for this display instance in the registry.
   *
   * @return Source key string (typically derived from channel).
   */
  std::string sourceId() const;

  /**
   * @brief Feedback topic used when publishing interaction feedback.
   *
   * @return Feedback channel name (property or
   *         @ref defaultFeedbackChannel).
   */
  std::string feedbackChannel() const;

  /** Shared store for Interact tool; non-owning. */
  InteractiveMarkerRegistry* registry_ = nullptr;

  /** Local name → marker state for this update channel. */
  std::map<std::string, InteractiveMarkerState> markers_;

  /** Optional reader for InteractiveMarkerInit snapshots. */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> init_reader_;

  /** Queue for init message payloads. */
  integration::MessageQueue init_queue_;
};

}  // namespace display
}  // namespace autoviz
