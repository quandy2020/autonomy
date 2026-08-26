/*
 * filter_chain.hpp
 *
 * Simple sequential filter chain (ROS-free replacement for filters::FilterChain).
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "autonomy/map/grid_map/grid_map_filters/filters/filter_base.hpp"

namespace filters {

template <typename T>
class FilterChain {
 public:
  FilterChain() = default;

  void clear() { filters_.clear(); }

  void add(std::unique_ptr<FilterBase<T>> filter) {
    filters_.push_back(std::move(filter));
  }

  /*!
   * Load a YAML sequence:
   *   - name: foo
   *     type: grid_map/ThresholdFilter
   *     params: { ... }
   * Factory must create the filter instance for `type`.
   */
  template <typename Factory>
  bool configure(const YAML::Node& chain, Factory&& factory) {
    clear();
    if (!chain || !chain.IsSequence()) {
      return false;
    }
    for (const auto& entry : chain) {
      const std::string type = entry["type"] ? entry["type"].as<std::string>() : "";
      const std::string name = entry["name"] ? entry["name"].as<std::string>() : type;
      auto filter = factory(type);
      if (!filter) {
        return false;
      }
      YAML::Node params = entry["params"] ? entry["params"] : YAML::Node(YAML::NodeType::Map);
      if (!filter->configure(params, name, type)) {
        return false;
      }
      filters_.push_back(std::move(filter));
    }
    return true;
  }

  bool update(const T& data_in, T& data_out) {
    if (filters_.empty()) {
      data_out = data_in;
      return true;
    }
    T tmp_in = data_in;
    T tmp_out;
    for (size_t i = 0; i < filters_.size(); ++i) {
      if (!filters_[i]->update(tmp_in, tmp_out)) {
        return false;
      }
      tmp_in = std::move(tmp_out);
    }
    data_out = std::move(tmp_in);
    return true;
  }

  size_t size() const { return filters_.size(); }

 private:
  std::vector<std::unique_ptr<FilterBase<T>>> filters_;
};

}  // namespace filters
