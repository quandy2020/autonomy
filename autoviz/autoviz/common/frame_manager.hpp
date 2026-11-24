/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_manager.hpp
 * @brief Fixed-frame + TF lookup facade (RViz FrameManager subset).
 *
 * Displays and tools query poses relative to the fixed frame through this
 * class. Supports pause and time-sync modes when replaying bags.
 *
 * @see TransformationManager
 * @see transform::Buffer
 * @see VisualizationManager::fixedFrame
 */

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <QQuaternion>
#include <QVector3D>

namespace autoviz {
namespace transform {
class Buffer;
}

namespace common {

/**
 * @class FrameManager
 * @brief Resolves frame poses into the configured fixed frame.
 *
 * ## Time sync
 *
 * @ref SyncMode controls whether lookups use wall time, exact message stamps,
 * or approximate time (bag replay). @ref syncTime / @ref currentTimeSec expose
 * the active clock used for lookups.
 */
class FrameManager {
 public:
  /**
   * @enum SyncMode
   * @brief How transform lookups choose their query time.
   */
  enum class SyncMode {
    kOff = 0,     /**< Use latest / wall time (live). */
    kExact = 1,   /**< Exact stamp sync with the source topic. */
    kApprox = 2,  /**< Approximate time sync. */
  };

  /**
   * @brief Attaches the TF buffer used for lookups.
   * @param buffer Non-owning buffer; may be @c nullptr in identity mode.
   */
  void setBuffer(transform::Buffer* buffer) { buffer_ = buffer; }

  /**
   * @brief Enables identity mode (all frames coincide with the fixed frame).
   * @param identity @c true to skip TF lookups.
   */
  void setIdentityMode(bool identity) { identity_mode_ = identity; }

  /**
   * @brief Whether identity mode is active.
   * @return @c true if TF is bypassed.
   */
  bool identityMode() const { return identity_mode_; }

  /**
   * @brief Sets the fixed (world) frame name.
   * @param frame Frame id (e.g. @c "map").
   */
  void setFixedFrame(const std::string& frame);

  /**
   * @brief Returns the current fixed frame.
   * @return Fixed frame id.
   */
  const std::string& fixedFrame() const { return fixed_frame_; }

  /**
   * @brief Pauses / resumes time advancement for lookups.
   * @param pause @c true to freeze @ref currentTimeSec.
   */
  void setPause(bool pause) { paused_ = pause; }

  /**
   * @brief Whether time is paused.
   * @return Pause state.
   */
  bool paused() const { return paused_; }

  /**
   * @brief Sets the time synchronization mode.
   * @param mode Sync strategy.
   */
  void setSyncMode(SyncMode mode) { sync_mode_ = mode; }

  /**
   * @brief Returns the current sync mode.
   * @return Active @ref SyncMode.
   */
  SyncMode syncMode() const { return sync_mode_; }

  /**
   * @brief Supplies an external synchronized time (seconds).
   * @param sec Simulation / bag time in seconds.
   */
  void syncTime(double sec);

  /**
   * @brief Returns the time used for TF lookups.
   * @return Seconds (synced or wall-based depending on mode/pause).
   */
  double currentTimeSec() const;

  /**
   * @brief Looks up @p frame's pose in the fixed frame.
   *
   * @param frame Source frame id.
   * @param[out] position Translation into the fixed frame.
   * @param[out] orientation Rotation into the fixed frame.
   * @return @c true on success.
   */
  bool getTransform(const std::string& frame, QVector3D* position,
                    QQuaternion* orientation) const;

  /**
   * @brief Reports whether transforming @p frame into the fixed frame fails.
   *
   * @param frame Frame to test.
   * @param[out] error Optional human-readable error string.
   * @return @c true if there is a problem (transform unavailable).
   */
  bool transformHasProblems(const std::string& frame,
                              std::string* error) const;

  /**
   * @brief Reports whether @p frame itself is missing / invalid in the buffer.
   *
   * @param frame Frame to test.
   * @param[out] error Optional human-readable error string.
   * @return @c true if the frame has problems.
   */
  bool frameHasProblems(const std::string& frame, std::string* error) const;

  /**
   * @brief Lists all frame ids currently known to the buffer.
   * @return Frame name vector (empty in identity mode / no buffer).
   */
  std::vector<std::string> allFrameNames() const;

  /**
   * @brief Periodic update: freezes the lookup time for this tick.
   *
   * Displays query the same frame several times per tick (RobotModel links,
   * overlays, tools). Reusing one timestamp keeps those lookups cacheable and
   * consistent within a frame.
   */
  void update();

 private:
  transform::Buffer* buffer_ = nullptr;       /**< Non-owning TF buffer. */
  bool identity_mode_ = false;                /**< Bypass TF when true. */
  std::string fixed_frame_ = "map";           /**< World frame id. */
  bool paused_ = false;                       /**< Freeze lookup time. */
  SyncMode sync_mode_ = SyncMode::kOff;       /**< Time sync strategy. */
  double synced_time_sec_ = 0.0;              /**< Last @ref syncTime value. */
  double wall_origin_sec_ = 0.0;              /**< Wall-clock origin. */

  /**
   * @brief One resolved frame pose for the current tick.
   *
   * Failures are cached too so a missing frame does not retry (and throw)
   * once per display per tick.
   */
  struct CachedPose {
    bool ok = false;
    QVector3D position;
    QQuaternion orientation;
  };

  /** Lookup time frozen by @ref update(); NaN until the first tick. */
  double cache_time_sec_ = std::numeric_limits<double>::quiet_NaN();

  /** TF buffer revision the cache was built against. */
  mutable std::uint64_t cache_revision_ = 0;

  /** Per-frame pose cache for the current tick (cleared by @ref update()). */
  mutable std::unordered_map<std::string, CachedPose> pose_cache_;
};

}  // namespace common
}  // namespace autoviz
