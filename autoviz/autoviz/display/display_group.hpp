/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_group.hpp
 * @brief Nested display container equivalent to @c rviz_common::DisplayGroup.
 *
 * Groups appear as folders in the Displays tree and forward enable, update,
 * draw, reset, and config load/save to owned child @ref Display instances.
 *
 * @see Display
 * @see FailedDisplay
 * @see common::DisplayConfig
 */

#pragma once

#include <memory>
#include <vector>

#include "autoviz/display/display.hpp"

namespace autoviz {
namespace display {

/**
 * @class DisplayGroup
 * @brief Owning container for nested displays (RViz DisplayGroup analogue).
 *
 * Children are stored in tree order. @ref onEnable / @ref onDisable /
 * @ref onUpdate / @ref onDraw iterate @c children_; config save embeds each
 * child's @ref Display::saveToConfig under the group entry.
 *
 * @note The group itself draws nothing beyond forwarding @ref onDraw to
 *       children.
 *
 * @see Display
 */
class DisplayGroup : public Display {
 public:
  /**
   * @brief Catalog type id for group nodes.
   *
   * @return Always @c "Group".
   */
  std::string typeId() const override { return "Group"; }

  /**
   * @brief Appends a child display, taking ownership.
   *
   * @param child Non-null unique_ptr to insert at the end.
   * @see insertChild()
   */
  void addChild(std::unique_ptr<Display> child);

  /**
   * @brief Inserts a child at @p index, taking ownership.
   *
   * @param index Insertion index (clamped / as implemented in .cpp).
   * @param child Non-null unique_ptr to insert.
   */
  void insertChild(std::size_t index, std::unique_ptr<Display> child);

  /**
   * @brief Removes and returns the child at @p index.
   *
   * @param index Child index.
   * @return Ownership of the removed display.
   */
  std::unique_ptr<Display> takeChild(std::size_t index);

  /**
   * @brief Returns the owned child list (tree order).
   *
   * @return Const reference to @c children_.
   */
  const std::vector<std::unique_ptr<Display>>& children() const {
    return children_;
  }

  /**
   * @brief Mutable child accessor.
   *
   * @param index Child index.
   * @return Raw pointer, or @c nullptr if out of range (implementation-
   *         defined).
   */
  Display* child(std::size_t index);

  /**
   * @brief Const child accessor.
   *
   * @param index Child index.
   * @return Const raw pointer to the child.
   */
  const Display* child(std::size_t index) const;

  /**
   * @brief Resets this group and every child display.
   */
  void reset() override;

  /**
   * @brief Loads group + nested child Config trees.
   *
   * @param config Config node for this group.
   */
  void load(const common::Config& config) override;

  /**
   * @brief Saves group + nested child Config trees.
   *
   * @param config Mutable config node.
   */
  void save(common::Config config) const override;

  /**
   * @brief Serializes group and children into session config.
   *
   * @param config Output @ref common::DisplayConfig; must be non-null.
   */
  void saveToConfig(common::DisplayConfig* config) const override;

 protected:
  /**
   * @brief Enables all child displays (forwards enable semantics).
   */
  void onEnable() override;

  /**
   * @brief Disables all child displays.
   */
  void onDisable() override;

  /**
   * @brief Updates every child.
   */
  void onUpdate() override;

  /**
   * @brief Draws every enabled child into @p scene.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /** Owned nested displays in Displays-tree order. */
  std::vector<std::unique_ptr<Display>> children_;
};

}  // namespace display
}  // namespace autoviz
