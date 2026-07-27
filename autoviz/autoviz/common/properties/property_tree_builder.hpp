/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * Fluent builder for rviz-style grouped Display property trees.
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/properties/property.hpp"

namespace autoviz {
namespace common {

class PropertyTreeBuilder {
 public:
  using Getter =
      std::function<std::string(const std::string& key, const std::string& default_value)>;

  explicit PropertyTreeBuilder(Getter getter);

  Property* root() { return root_.get(); }
  Property* addGroup(const std::string& name, const std::string& label,
                     const std::string& description = {});

  void addSpec(Property* parent, const DisplayPropertySpec& spec);
  void addString(Property* parent, const std::string& key,
                 const std::string& label, const std::string& default_value);
  void addBool(Property* parent, const std::string& key,
               const std::string& label, const std::string& default_value);
  void addFloat(Property* parent, const std::string& key,
                const std::string& label, const std::string& default_value);
  void addInt(Property* parent, const std::string& key, const std::string& label,
              const std::string& default_value);
  void addColor(Property* parent, const std::string& key,
                const std::string& label, const std::string& default_value);
  void addPath(Property* parent, const std::string& key,
               const std::string& label, const std::string& default_value);
  void addChannel(Property* parent, const std::string& key,
                  const std::string& label, const std::string& default_value);
  void addEnum(Property* parent, const std::string& key,
               const std::string& label, const std::string& default_value,
               std::vector<std::string> options);

  std::unique_ptr<Property> takeRoot() { return std::move(root_); }

 private:
  std::string value(const std::string& key, const std::string& default_value) const;

  Getter getter_;
  std::unique_ptr<Property> root_;
};

}  // namespace common
}  // namespace autoviz
