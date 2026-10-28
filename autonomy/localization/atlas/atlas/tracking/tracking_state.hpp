#pragma once

#include <string_view>

namespace autonomy::localization::atlas {

/** @brief Strongly typed view of the legacy ORB-SLAM3 tracking state. */
enum class TrackingState : int {
    kSystemNotReady = -1,
    kNoImagesYet = 0,
    kNotInitialized = 1,
    kOk = 2,
    kRecentlyLost = 3,
    kLost = 4,
    kOkKlt = 5,
};

/** @brief Converts an ORB-SLAM3 integer state to TrackingState. */
TrackingState TrackingStateFromLegacy(int state);
/** @return Stable lowercase name for logging and diagnostics. */
std::string_view ToString(TrackingState state);

}  // namespace autonomy::localization::atlas
