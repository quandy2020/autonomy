/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/properties/property_tree_builder.hpp"

#include "autoviz/common/properties/property_factory.hpp"

namespace autoviz {
namespace common {

PropertyTreeBuilder::PropertyTreeBuilder(Getter getter)
    : getter_(std::move(getter)), root_(std::make_unique<Property>("", "")) {}

Property* PropertyTreeBuilder::addGroup(const std::string& name,
                                        const std::string& label,
                                        const std::string& description) {
  auto group = std::make_unique<Property>(name, label);
  group->setDescription(description);
  return root_->addChild(std::move(group));
}

void PropertyTreeBuilder::addSpec(Property* parent,
                                  const DisplayPropertySpec& spec) {
  if (parent == nullptr) {
    return;
  }
  parent->addChild(
      CreatePropertyFromSpec(spec, value(spec.key, spec.default_value)));
}

void PropertyTreeBuilder::addString(Property* parent, const std::string& key,
                                    const std::string& label,
                                    const std::string& default_value) {
  if (parent == nullptr) {
    return;
  }
  parent->addChild(
      std::make_unique<StringProperty>(key, label, value(key, default_value)));
}

void PropertyTreeBuilder::addBool(Property* parent, const std::string& key,
                                  const std::string& label,
                                  const std::string& default_value) {
  if (parent == nullptr) {
    return;
  }
  parent->addChild(std::make_unique<BoolProperty>(
      key, label, ParseBoolProperty(value(key, default_value), default_value == "true")));
}

void PropertyTreeBuilder::addFloat(Property* parent, const std::string& key,
                                   const std::string& label,
                                   const std::string& default_value) {
  if (parent == nullptr) {
    return;
  }
  parent->addChild(std::make_unique<FloatProperty>(
      key, label, ParseFloatProperty(value(key, default_value),
                                     ParseFloatProperty(default_value, 0.f))));
}

void PropertyTreeBuilder::addInt(Property* parent, const std::string& key,
                                 const std::string& label,
                                 const std::string& default_value) {
  if (parent == nullptr) {
    return;
  }
  parent->addChild(std::make_unique<IntProperty>(
      key, label,
      static_cast<int>(ParseFloatProperty(value(key, default_value),
                                          ParseFloatProperty(default_value, 0.f)))));
}

void PropertyTreeBuilder::addColor(Property* parent, const std::string& key,
                                   const std::string& label,
                                   const std::string& default_value) {
  if (parent == nullptr) {
    return;
  }
  parent->addChild(
      std::make_unique<ColorProperty>(key, label, value(key, default_value)));
}

void PropertyTreeBuilder::addPath(Property* parent, const std::string& key,
                                const std::string& label,
                                const std::string& default_value) {
  addString(parent, key, label, default_value);
}

void PropertyTreeBuilder::addChannel(Property* parent, const std::string& key,
                                     const std::string& label,
                                     const std::string& default_value) {
  addString(parent, key, label, default_value);
}

void PropertyTreeBuilder::addEnum(Property* parent, const std::string& key,
                                  const std::string& label,
                                  const std::string& default_value,
                                  std::vector<std::string> options) {
  if (parent == nullptr) {
    return;
  }
  parent->addChild(std::make_unique<EnumProperty>(
      key, label, value(key, default_value), std::move(options)));
}

std::string PropertyTreeBuilder::value(const std::string& key,
                                       const std::string& default_value) const {
  return getter_(key, default_value);
}

}  // namespace common
}  // namespace autoviz
