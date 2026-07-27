/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * rviz_common::properties::Property — hierarchical display settings (zero ROS).
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/display_property.hpp"

namespace autoviz {
namespace common {

/** Base node: group (non-empty children) or leaf (value-bearing subclass). */
class Property {
 public:
  using ChangedCallback = std::function<void(Property*)>;

  Property(std::string name, std::string label, Property* parent = nullptr);
  virtual ~Property() = default;

  const std::string& name() const { return name_; }
  const std::string& label() const { return label_; }
  const std::string& description() const { return description_; }
  void setDescription(std::string description) { description_ = std::move(description); }

  Property* parent() const { return parent_; }
  const std::vector<std::unique_ptr<Property>>& children() const { return children_; }
  bool isGroup() const { return !children_.empty(); }

  Property* addChild(std::unique_ptr<Property> child);
  Property* findChild(const std::string& name) const;
  Property* findLeaf(const std::string& name) const;

  void setChangedCallback(ChangedCallback callback);

  virtual std::string valueString() const;
  virtual void setValueString(const std::string& value);
  virtual DisplayPropertyKind editorKind() const { return DisplayPropertyKind::kAuto; }
  virtual std::vector<std::string> enumOptions() const { return {}; }

 protected:
  void notifyChanged();

  std::string name_;
  std::string label_;
  std::string description_;
  Property* parent_ = nullptr;
  std::vector<std::unique_ptr<Property>> children_;
  ChangedCallback changed_callback_;
};

class StringProperty : public Property {
 public:
  StringProperty(std::string name, std::string label, std::string value = {},
                 Property* parent = nullptr);

  std::string valueString() const override;
  void setValueString(const std::string& value) override;

 private:
  std::string value_;
};

class BoolProperty : public Property {
 public:
  BoolProperty(std::string name, std::string label, bool value = false,
               Property* parent = nullptr);

  std::string valueString() const override;
  void setValueString(const std::string& value) override;
  DisplayPropertyKind editorKind() const override { return DisplayPropertyKind::kAuto; }

 private:
  bool value_ = false;
};

class FloatProperty : public Property {
 public:
  FloatProperty(std::string name, std::string label, float value = 0.f,
                Property* parent = nullptr);

  std::string valueString() const override;
  void setValueString(const std::string& value) override;

 private:
  float value_ = 0.f;
};

class IntProperty : public Property {
 public:
  IntProperty(std::string name, std::string label, int value = 0,
              Property* parent = nullptr);

  std::string valueString() const override;
  void setValueString(const std::string& value) override;

 private:
  int value_ = 0;
};

class ColorProperty : public Property {
 public:
  ColorProperty(std::string name, std::string label, std::string value = "200;200;200",
                Property* parent = nullptr);

  std::string valueString() const override;
  void setValueString(const std::string& value) override;
  DisplayPropertyKind editorKind() const override { return DisplayPropertyKind::kColor; }

 private:
  std::string value_;
};

class EnumProperty : public Property {
 public:
  EnumProperty(std::string name, std::string label, std::string value,
               std::vector<std::string> options, Property* parent = nullptr);

  std::string valueString() const override;
  void setValueString(const std::string& value) override;
  std::vector<std::string> enumOptions() const override { return options_; }

 private:
  std::string value_;
  std::vector<std::string> options_;
};

}  // namespace common
}  // namespace autoviz
