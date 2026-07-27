/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"

namespace autoviz {
namespace integration {

class AutolinkContext {
 public:
  AutolinkContext() = default;
  ~AutolinkContext();

  AutolinkContext(const AutolinkContext&) = delete;
  AutolinkContext& operator=(const AutolinkContext&) = delete;

  bool initialize(const char* binary_name, const std::string& node_name);
  void shutdown();

  bool ok() const;
  std::shared_ptr<::autolink::Node> node() const { return node_; }

 private:
  std::shared_ptr<::autolink::Node> node_;
  bool initialized_ = false;
};

}  // namespace integration
}  // namespace autoviz
