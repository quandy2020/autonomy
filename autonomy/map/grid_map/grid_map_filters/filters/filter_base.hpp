/*
 * filter_base.hpp
 *
 * ROS-free FilterBase for grid_map filters (YAML-backed parameters).
 * API mirrors ros/filters FilterBase enough for grid_map_filters ports.
 */

#pragma once

#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace filters {

template <typename T>
class FilterBase {
 public:
  FilterBase() : configured_(false) {}
  virtual ~FilterBase() = default;

  /*!
   * Configure from a YAML map of parameters (flat or nested).
   * Nested keys can be read with slash paths, e.g. "min/value".
   */
  bool configure(const YAML::Node& params,
                 const std::string& name = "",
                 const std::string& type = "") {
    if (configured_) {
      // Allow reconfigure.
    }
    configured_ = false;
    filter_name_ = name;
    filter_type_ = type;
    params_ = params.IsNull() ? YAML::Node(YAML::NodeType::Map) : params;
    const bool ok = configure();
    configured_ = ok;
    return ok;
  }

  bool configureFromFile(const std::string& filename,
                         const std::string& name = "",
                         const std::string& type = "") {
    return configure(YAML::LoadFile(filename), name, type);
  }

  virtual bool update(const T& data_in, T& data_out) = 0;

  std::string getType() const { return filter_type_; }
  const std::string& getName() const { return filter_name_; }
  bool isConfigured() const { return configured_; }

 protected:
  virtual bool configure() = 0;

  YAML::Node lookupParam(const std::string& name) const {
    if (!params_ || !params_.IsMap()) {
      return YAML::Node();
    }
    // Exact key (supports literal "min/value").
    if (params_[name]) {
      return params_[name];
    }
    // Nested path via '/'.
    YAML::Node cur = params_;
    std::stringstream ss(name);
    std::string part;
    while (std::getline(ss, part, '/')) {
      if (!cur || !cur.IsMap() || !cur[part]) {
        return YAML::Node();
      }
      cur = cur[part];
    }
    return cur;
  }

  bool getParam(const std::string& name, std::string& value) const {
    const YAML::Node node = lookupParam(name);
    if (!node || !node.IsScalar()) {
      return false;
    }
    value = node.as<std::string>();
    return true;
  }

  bool getParam(const std::string& name, bool& value) const {
    const YAML::Node node = lookupParam(name);
    if (!node || !node.IsScalar()) {
      return false;
    }
    value = node.as<bool>();
    return true;
  }

  bool getParam(const std::string& name, double& value) const {
    const YAML::Node node = lookupParam(name);
    if (!node || !node.IsScalar()) {
      return false;
    }
    value = node.as<double>();
    return true;
  }

  bool getParam(const std::string& name, int& value) const {
    const YAML::Node node = lookupParam(name);
    if (!node || !node.IsScalar()) {
      return false;
    }
    value = node.as<int>();
    return true;
  }

  bool getParam(const std::string& name, unsigned int& value) const {
    int signed_value = 0;
    if (!getParam(name, signed_value) || signed_value < 0) {
      return false;
    }
    value = static_cast<unsigned int>(signed_value);
    return true;
  }

  bool getParam(const std::string& name, std::vector<double>& value) const {
    const YAML::Node node = lookupParam(name);
    if (!node || !node.IsSequence()) {
      return false;
    }
    value.clear();
    value.reserve(node.size());
    for (const auto& item : node) {
      value.push_back(item.as<double>());
    }
    return true;
  }

  bool getParam(const std::string& name, std::vector<std::string>& value) const {
    const YAML::Node node = lookupParam(name);
    if (!node || !node.IsSequence()) {
      return false;
    }
    value.clear();
    value.reserve(node.size());
    for (const auto& item : node) {
      value.push_back(item.as<std::string>());
    }
    return true;
  }

  std::string filter_name_;
  std::string filter_type_;
  bool configured_;
  YAML::Node params_;
};

}  // namespace filters
