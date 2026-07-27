/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

namespace autoviz {
namespace rendering {

/** Detects hardware GPU; gates Ogre backend and GPU depth picking. */
class GpuCapabilities {
 public:
  static GpuCapabilities& instance();

  void probeFromOpenGL();
  void probeFromRendererString(const std::string& renderer);

  bool probed() const { return probed_; }
  bool hasHardwareGpu() const { return has_hardware_gpu_; }
  const std::string& rendererName() const { return renderer_name_; }

  /** One-time offscreen GL probe when no viewport exists yet. */
  void ensureProbed();

 private:
  GpuCapabilities() = default;

  static bool IsSoftwareRenderer(const std::string& renderer);

  bool probed_ = false;
  bool has_hardware_gpu_ = false;
  std::string renderer_name_;
};

}  // namespace rendering
}  // namespace autoviz
