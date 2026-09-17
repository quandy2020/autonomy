/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/constraints/constraint_sampler_manager.hpp"

#include <fstream>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/plugin_ids.hpp"

namespace autonomy {
namespace manipulation {
namespace constraints {

void ConstraintSamplerManager::LoadRegisteredAllocators() {
  allocators_.clear();
  RegisterManipulationPlugins();
  static const char* kNames[] = {
      "UnionConstraintSamplerAllocator",
      "IkConstraintSamplerAllocator",
      "JointConstraintSamplerAllocator",
  };
  for (const char* name : kNames) {
    auto a = CreatePlugin<ConstraintSamplerAllocator>(name);
    if (a) {
      RegisterAllocator(std::move(a));
    }
  }
  if (allocators_.empty()) {
    AWARN << "ConstraintSamplerManager: no allocator plugins; using builtins";
    RegisterAllocator(std::make_shared<UnionConstraintSamplerAllocator>());
    RegisterAllocator(std::make_shared<IkConstraintSamplerAllocator>());
    RegisterAllocator(std::make_shared<JointConstraintSamplerAllocator>());
  } else {
    AINFO << "ConstraintSamplerManager loaded " << allocators_.size()
          << " allocator plugins";
  }
}

int ConstraintSamplerManager::LoadExternalPluginDescriptions(
    const std::string& plugins_list_file) {
  std::string path = plugins_list_file;
  if (path.empty()) {
    ::autonomy::common::ResolveModuleConfPath("manipulation", "constraint_samplers.plugins",
                                  &path);
  }
  if (path.empty()) {
    return 0;
  }
  std::ifstream in(path);
  if (!in) {
    return 0;
  }
  auto* pm = autolink::plugin_manager::PluginManager::Instance();
  int loaded = 0;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    while (!line.empty() && (line.back() == ' ' || line.back() == '\t' ||
                             line.back() == '\r')) {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    if (pm->LoadPlugin(line)) {
      ++loaded;
      AINFO << "ConstraintSamplerManager loaded external plugin desc=" << line;
    } else {
      AWARN << "ConstraintSamplerManager failed to load plugin desc=" << line;
    }
  }
  if (loaded > 0) {
    LoadRegisteredAllocators();
  }
  return loaded;
}

}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
