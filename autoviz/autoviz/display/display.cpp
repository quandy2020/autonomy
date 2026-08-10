/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/display.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_scene_host.hpp"
#endif

namespace autoviz {
namespace display {

void Display::setProperties(const common::DisplayPropertyMap& properties) {
  properties_ = properties;
  for (const auto& spec : propertySpecs()) {
    if (properties_.find(spec.key) == properties_.end()) {
      properties_[spec.key] = spec.default_value;
    }
  }
}

std::string Display::propertyValue(const std::string& key,
                                   const std::string& default_value) const {
  const auto it = properties_.find(key);
  return it != properties_.end() ? it->second : default_value;
}

void Display::setPropertyValue(const std::string& key,
                               const std::string& value) {
  if (properties_[key] == value) {
    return;
  }
  properties_[key] = value;
  onPropertyChanged(key);
}

void Display::setVisibilityBits(uint32_t bits) {
  visibility_bits_ |= bits;
#ifdef AUTOVIZ_USE_OGRE
  if (context_ != nullptr && context_->ogre_scene_host != nullptr) {
    context_->ogre_scene_host->setDisplayVisibilityBits(name(), visibility_bits_);
  }
#endif
}

void Display::unsetVisibilityBits(uint32_t bits) {
  visibility_bits_ &= ~bits;
#ifdef AUTOVIZ_USE_OGRE
  if (context_ != nullptr && context_->ogre_scene_host != nullptr) {
    context_->ogre_scene_host->setDisplayVisibilityBits(name(), visibility_bits_);
  }
#endif
}

void Display::setEnabled(bool enabled) {
  if (enabled_ == enabled) {
    return;
  }
  enabled_ = enabled;
  if (enabled_) {
    if (context_ != nullptr) {
      setVisibilityBits(context_->default_visibility_bit);
    }
    onEnable();
    subscribed_ = true;
  } else if (subscribed_) {
    onDisable();
    subscribed_ = false;
#ifdef AUTOVIZ_USE_OGRE
    if (context_ != nullptr && context_->ogre_scene_host != nullptr) {
      context_->ogre_scene_host->removeDisplaysWithPrefix(name());
    }
#endif
  }
  if (!enabled_) {
    setStatusOk();
  }
}

void Display::resetStatus() {
  status_level_ = DisplayStatusLevel::kOk;
  status_message_.clear();
}

void Display::setStatusOk() { resetStatus(); }

void Display::setStatusOk(const std::string& message) {
  status_level_ = DisplayStatusLevel::kOk;
  status_message_ = message;
}

void Display::setStatusWarn(const std::string& message) {
  status_level_ = DisplayStatusLevel::kWarn;
  status_message_ = message;
}

void Display::setStatusError(const std::string& message) {
  status_level_ = DisplayStatusLevel::kError;
  status_message_ = message;
}

void Display::update() {
  if (!enabled_) {
    return;
  }
  resetStatus();
  onUpdate();
}

void Display::draw(rendering::SceneOverlay& scene) {
  if (!enabled_) {
    return;
  }
  const std::string active_name = name();
  const std::string active_type = typeId();
  if (context_ != nullptr) {
    context_->active_display_name = &active_name;
    context_->active_display_type = &active_type;
    context_->active_display_visibility_bits = &visibility_bits_;
  }
  scene.setPickSource(&active_name, &active_type);
  onDraw(scene);
}

void Display::loadFromConfig(const common::DisplayConfig& config) {
  display_name_ = config.name.empty() ? typeId() : config.name;
  setChannel(config.channel);
  setProperties(config.properties);
  setEnabled(config.enabled);
}

void Display::saveToConfig(common::DisplayConfig* config) const {
  if (config == nullptr) {
    return;
  }
  config->type = typeId();
  config->name = name();
  config->channel = channel();
  config->enabled = enabled_;
  config->properties = properties_;
  config->children.clear();
}

void Display::load(const common::Config& config) {
  common::DisplayConfig entry;
  QString value;
  if (config.mapGetString("Type", &value)) {
    entry.type = value.toStdString();
  } else {
    entry.type = typeId();
  }
  if (config.mapGetString("Name", &value)) {
    entry.name = value.toStdString();
  }
  if (config.mapGetString("Channel", &value)) {
    entry.channel = value.toStdString();
  }
  bool enabled = enabled_;
  if (config.mapGetBool("Enabled", &enabled)) {
    entry.enabled = enabled;
  } else {
    entry.enabled = enabled_;
  }
  entry.properties.clear();
  const common::Config props = config.mapGetChild("Properties");
  if (props.isValid()) {
    for (common::Config::MapIterator it = props.mapIterator(); it.isValid();
         it.advance()) {
      const common::Config child = it.currentChild();
      if (child.getType() == common::Config::Value) {
        entry.properties[it.currentKey().toStdString()] =
            child.getValue().toString().toStdString();
      }
    }
  }
  loadFromConfig(entry);
}

void Display::save(common::Config config) const {
  config.mapSetValue("Type", QString::fromStdString(typeId()));
  config.mapSetValue("Name", QString::fromStdString(name()));
  config.mapSetValue("Channel", QString::fromStdString(channel()));
  config.mapSetValue("Enabled", enabled_);
  if (!properties_.empty()) {
    common::Config props = config.mapMakeChild("Properties");
    for (const auto& prop : properties_) {
      props.mapSetValue(QString::fromStdString(prop.first),
                        QString::fromStdString(prop.second));
    }
  }
}

}  // namespace display
}  // namespace autoviz
