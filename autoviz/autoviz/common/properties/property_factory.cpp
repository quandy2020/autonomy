/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/properties/property_factory.hpp"

namespace autoviz {
namespace common {
namespace {

std::string ResolveValue(const DisplayPropertySpec& spec,
                         const DisplayPropertyMap& values) {
  const auto it = values.find(spec.key);
  if (it != values.end()) {
    return it->second;
  }
  return spec.default_value;
}

}  // namespace

std::unique_ptr<Property> CreatePropertyFromSpec(const DisplayPropertySpec& spec,
                                                 const std::string& value) {
  if (!spec.options.empty()) {
    return std::make_unique<EnumProperty>(spec.key, spec.label, value,
                                          spec.options);
  }
  switch (spec.kind) {
    case DisplayPropertyKind::kColor:
      return std::make_unique<ColorProperty>(spec.key, spec.label, value);
    case DisplayPropertyKind::kPath:
    case DisplayPropertyKind::kChannel:
      return std::make_unique<StringProperty>(spec.key, spec.label, value);
    case DisplayPropertyKind::kAuto:
    default:
      break;
  }
  const std::string lower = spec.default_value;
  if (lower == "true" || lower == "false") {
    return std::make_unique<BoolProperty>(
        spec.key, spec.label, ParseBoolProperty(value, lower == "true"));
  }
  if (spec.key.find("color") != std::string::npos) {
    return std::make_unique<ColorProperty>(spec.key, spec.label, value);
  }
  return std::make_unique<StringProperty>(spec.key, spec.label, value);
}

std::unique_ptr<Property> BuildPropertyTreeFromSpecs(
    const std::vector<DisplayPropertySpec>& specs,
    const DisplayPropertyMap& values) {
  auto root = std::make_unique<Property>("", "");
  for (const auto& spec : specs) {
    root->addChild(
        CreatePropertyFromSpec(spec, ResolveValue(spec, values)));
  }
  return root;
}

void SyncPropertyTreeFromMap(Property* root, const DisplayPropertyMap& values) {
  if (root == nullptr) {
    return;
  }
  for (const auto& child : root->children()) {
    if (child->isGroup()) {
      SyncPropertyTreeFromMap(child.get(), values);
      continue;
    }
    const auto it = values.find(child->name());
    if (it != values.end()) {
      child->setValueString(it->second);
    }
  }
}

void SyncPropertyTreeToMap(const Property& root, DisplayPropertyMap* values) {
  if (values == nullptr) {
    return;
  }
  for (const auto& child : root.children()) {
    if (child->isGroup()) {
      SyncPropertyTreeToMap(*child, values);
      continue;
    }
    if (!child->name().empty()) {
      (*values)[child->name()] = child->valueString();
    }
  }
}

}  // namespace common
}  // namespace autoviz
