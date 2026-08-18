/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/display_group.hpp"

namespace autoviz {
namespace display {

void DisplayGroup::addChild(std::unique_ptr<Display> child) {
  insertChild(children_.size(), std::move(child));
}

void DisplayGroup::insertChild(std::size_t index,
                               std::unique_ptr<Display> child) {
  if (child == nullptr) {
    return;
  }
  child->setContext(context_);
  child->setEnabled(enabled() && child->enabled());
  if (index >= children_.size()) {
    children_.push_back(std::move(child));
    return;
  }
  children_.insert(children_.begin() + static_cast<std::ptrdiff_t>(index),
                   std::move(child));
}

std::unique_ptr<Display> DisplayGroup::takeChild(std::size_t index) {
  if (index >= children_.size()) {
    return nullptr;
  }
  auto child =
      std::move(children_[static_cast<std::ptrdiff_t>(index)]);
  children_.erase(children_.begin() + static_cast<std::ptrdiff_t>(index));
  return child;
}

Display* DisplayGroup::child(std::size_t index) {
  return index < children_.size() ? children_[index].get() : nullptr;
}

const Display* DisplayGroup::child(std::size_t index) const {
  return index < children_.size() ? children_[index].get() : nullptr;
}

void DisplayGroup::onEnable() {
  for (auto& child : children_) {
    child->setEnabled(true);
  }
}

void DisplayGroup::onDisable() {
  for (auto& child : children_) {
    child->setEnabled(false);
  }
}

void DisplayGroup::onUpdate() {
  for (auto& child : children_) {
    if (child->enabled()) {
      child->update();
    }
  }
}

void DisplayGroup::onDraw(rendering::SceneOverlay& scene) {
  for (auto& child : children_) {
    if (child->enabled()) {
      child->draw(scene);
    }
  }
}

void DisplayGroup::reset() {
  Display::reset();
  for (auto& child : children_) {
    child->reset();
  }
}

void DisplayGroup::load(const common::Config& config) {
  Display::load(config);
}

void DisplayGroup::save(common::Config config) const {
  Display::save(config);
  if (!children_.empty()) {
    common::Config children = config.mapMakeChild("Children");
    for (const auto& child : children_) {
      common::Config node = children.listAppendNew();
      child->save(node);
    }
  }
}

void DisplayGroup::saveToConfig(common::DisplayConfig* config) const {
  Display::saveToConfig(config);
  if (config == nullptr) {
    return;
  }
  config->children.clear();
  config->children.reserve(children_.size());
  for (const auto& child : children_) {
    common::DisplayConfig child_config;
    child->saveToConfig(&child_config);
    config->children.push_back(std::move(child_config));
  }
}

}  // namespace display
}  // namespace autoviz
