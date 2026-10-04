/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file gpu_capabilities.hpp
 * @brief Detect hardware GPU vs software renderers; gate Ogre / GPU picking.
 *
 * Singleton probed from an active OpenGL context or a renderer string.
 * Soft/LLVMpipe-style renderers disable Ogre backend preference and GPU depth
 * picking to avoid slow or broken paths.
 *
 * @see RenderSystem
 * @see pickScenePoint()
 * @see OgreRenderBackend
 */

#pragma once

#include <string>

namespace autoviz {
namespace rendering {

/**
 * @class GpuCapabilities
 * @brief Process-wide GPU capability probe.
 *
 * ## Usage
 *
 * - After the first GL context exists: @ref probeFromOpenGL().
 * - Before any viewport (tests / early init): @ref ensureProbed() creates a
 *   short-lived offscreen context.
 * - Or inject a known string via @ref probeFromRendererString().
 */
class GpuCapabilities {
 public:
  /**
   * @brief Returns the process singleton.
   * @return Mutable reference to the shared instance.
   */
  static GpuCapabilities& instance();

  /**
   * @brief Probes @c GL_RENDERER from the current OpenGL context.
   *
   * Sets @ref probed() and @ref hasHardwareGpu() based on
   * @ref IsSoftwareRenderer().
   */
  void probeFromOpenGL();

  /**
   * @brief Probes using an explicit renderer string (no GL context needed).
   *
   * @param renderer Value as returned by @c glGetString(GL_RENDERER).
   */
  void probeFromRendererString(const std::string& renderer);

  /**
   * @brief Whether a probe has completed.
   * @return @c true after any successful @c probe* / @ref ensureProbed().
   */
  bool probed() const { return probed_; }

  /**
   * @brief Whether the probed renderer looks like a hardware GPU.
   * @return @c false for software / null / empty renderers.
   */
  bool hasHardwareGpu() const { return has_hardware_gpu_; }

  /**
   * @brief Last probed @c GL_RENDERER string (may be empty).
   * @return Const reference to the stored name.
   */
  const std::string& rendererName() const { return renderer_name_; }

  /**
   * @brief Performs a one-time offscreen GL probe when no viewport exists yet.
   *
   * No-op if already @ref probed().
   */
  void ensureProbed();

 private:
  GpuCapabilities() = default;

  /**
   * @brief Heuristic: llvmpipe, softpipe, SwiftShader, etc.
   * @param renderer Renderer string to classify.
   * @return @c true if treated as software.
   */
  static bool IsSoftwareRenderer(const std::string& renderer);

  bool probed_ = false;              /**< Probe completed. */
  bool has_hardware_gpu_ = false;    /**< Hardware GPU detected. */
  std::string renderer_name_;        /**< Cached GL_RENDERER. */
};

}  // namespace rendering
}  // namespace autoviz
