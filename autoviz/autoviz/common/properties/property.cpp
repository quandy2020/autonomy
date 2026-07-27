/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/properties/property.hpp"

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace common {

Property::Property(std::string name, std::string label, Property* parent)
    : name_(std::move(name)), label_(std::move(label)), parent_(parent) {}

Property* Property::addChild(std::unique_ptr<Property> child) {
  if (child == nullptr) {
    return nullptr;
  }
  child->parent_ = this;
  children_.push_back(std::move(child));
  return children_.back().get();
}

Property* Property::findChild(const std::string& name) const {
  for (const auto& child : children_) {
    if (child->name() == name) {
      return child.get();
    }
    if (Property* nested = child->findChild(name)) {
      return nested;
    }
  }
  return nullptr;
}

Property* Property::findLeaf(const std::string& name) const {
  for (const auto& child : children_) {
    if (!child->isGroup() && child->name() == name) {
      return child.get();
    }
    if (Property* nested = child->findLeaf(name)) {
      return nested;
    }
  }
  return nullptr;
}

void Property::setChangedCallback(ChangedCallback callback) {
  changed_callback_ = std::move(callback);
  for (const auto& child : children_) {
    child->setChangedCallback(changed_callback_);
  }
}

std::string Property::valueString() const { return {}; }

void Property::setValueString(const std::string& /*value*/) {}

void Property::notifyChanged() {
  if (changed_callback_) {
    changed_callback_(this);
  }
}

StringProperty::StringProperty(std::string name, std::string label,
                               std::string value, Property* parent)
    : Property(std::move(name), std::move(label), parent),
      value_(std::move(value)) {}

std::string StringProperty::valueString() const { return value_; }

void StringProperty::setValueString(const std::string& value) {
  if (value_ == value) {
    return;
  }
  value_ = value;
  notifyChanged();
}

BoolProperty::BoolProperty(std::string name, std::string label, bool value,
                           Property* parent)
    : Property(std::move(name), std::move(label), parent), value_(value) {}

std::string BoolProperty::valueString() const { return value_ ? "true" : "false"; }

void BoolProperty::setValueString(const std::string& value) {
  const bool parsed = ParseBoolProperty(value, value_);
  if (parsed == value_) {
    return;
  }
  value_ = parsed;
  notifyChanged();
}

FloatProperty::FloatProperty(std::string name, std::string label, float value,
                             Property* parent)
    : Property(std::move(name), std::move(label), parent), value_(value) {}

std::string FloatProperty::valueString() const {
  const int as_int = static_cast<int>(value_);
  if (std::abs(value_ - static_cast<float>(as_int)) < 1e-4f) {
    return std::to_string(as_int);
  }
  return std::to_string(value_);
}

void FloatProperty::setValueString(const std::string& value) {
  const float parsed = ParseFloatProperty(value, value_);
  if (std::abs(parsed - value_) < 1e-6f) {
    return;
  }
  value_ = parsed;
  notifyChanged();
}

IntProperty::IntProperty(std::string name, std::string label, int value,
                         Property* parent)
    : Property(std::move(name), std::move(label), parent), value_(value) {}

std::string IntProperty::valueString() const { return std::to_string(value_); }

void IntProperty::setValueString(const std::string& value) {
  const int parsed = static_cast<int>(ParseFloatProperty(value, static_cast<float>(value_)));
  if (parsed == value_) {
    return;
  }
  value_ = parsed;
  notifyChanged();
}

ColorProperty::ColorProperty(std::string name, std::string label,
                             std::string value, Property* parent)
    : Property(std::move(name), std::move(label), parent),
      value_(std::move(value)) {}

std::string ColorProperty::valueString() const { return value_; }

void ColorProperty::setValueString(const std::string& value) {
  if (value_ == value) {
    return;
  }
  value_ = value;
  notifyChanged();
}

EnumProperty::EnumProperty(std::string name, std::string label,
                             std::string value,
                             std::vector<std::string> options,
                             Property* parent)
    : Property(std::move(name), std::move(label), parent),
      value_(std::move(value)),
      options_(std::move(options)) {}

std::string EnumProperty::valueString() const { return value_; }

void EnumProperty::setValueString(const std::string& value) {
  if (value_ == value) {
    return;
  }
  value_ = value;
  notifyChanged();
}

}  // namespace common
}  // namespace autoviz
