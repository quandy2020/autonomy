/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

namespace autoviz::widget {

// Process-wide preference used by widget animations. The system mode honors
// the Qt widget animation style hint and explicit environment preferences.
enum class MotionPreference {
  kSystem,
  kFullMotion,
  kReducedMotion,
};

// Thread-safe query facade for reduced-motion behavior. Preference changes are
// atomic. RefreshSystemPreference() should be called from the GUI thread after
// a style or environment change so worker-thread queries can use the cache.
class MotionPolicy final {
 public:
  MotionPolicy() = delete;

  static MotionPreference preference() noexcept;
  static void SetPreference(MotionPreference preference) noexcept;

  static bool ReducedMotionEnabled() noexcept;
  static int ResolveDuration(int duration_ms) noexcept;
  static void RefreshSystemPreference() noexcept;
};

}  // namespace autoviz::widget
