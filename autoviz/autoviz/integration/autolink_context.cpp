/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/autolink_context.hpp"

#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autolink/init.hpp"
#include "autolink/state.hpp"

namespace autoviz {
namespace integration {

AutolinkContext::~AutolinkContext() { shutdown(); }

bool AutolinkContext::initialize(const char* binary_name,
                                 const std::string& node_name) {
  if (initialized_) {
    return true;
  }
  if (!::autolink::Init(binary_name)) {
    LOG(ERROR) << "Failed to initialize autolink runtime.";
    return false;
  }
  node_ = ::autolink::CreateNode(node_name);
  if (node_ == nullptr) {
    LOG(ERROR) << "Failed to create autolink node: " << node_name;
    ::autolink::Clear();
    return false;
  }
  initialized_ = true;
  LOG(INFO) << "Autolink node ready: " << node_name;
  return true;
}

void AutolinkContext::shutdown() {
  if (!initialized_) {
    return;
  }
  node_.reset();
  ::autolink::Clear();
  initialized_ = false;
}

bool AutolinkContext::ok() const {
  return initialized_ && ::autolink::OK();
}

}  // namespace integration
}  // namespace autoviz
