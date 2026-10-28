#include "atlas/tracking/tracking_state.hpp"

namespace autonomy::localization::atlas {

TrackingState TrackingStateFromLegacy(int state) {
    switch (state) {
        case -1:
            return TrackingState::kSystemNotReady;
        case 0:
            return TrackingState::kNoImagesYet;
        case 1:
            return TrackingState::kNotInitialized;
        case 2:
            return TrackingState::kOk;
        case 3:
            return TrackingState::kRecentlyLost;
        case 4:
            return TrackingState::kLost;
        case 5:
            return TrackingState::kOkKlt;
        default:
            return TrackingState::kLost;
    }
}

std::string_view ToString(TrackingState state) {
    switch (state) {
        case TrackingState::kSystemNotReady:
            return "system_not_ready";
        case TrackingState::kNoImagesYet:
            return "no_images_yet";
        case TrackingState::kNotInitialized:
            return "not_initialized";
        case TrackingState::kOk:
            return "ok";
        case TrackingState::kRecentlyLost:
            return "recently_lost";
        case TrackingState::kLost:
            return "lost";
        case TrackingState::kOkKlt:
            return "ok_klt";
    }
    return "unknown";
}

}  // namespace autonomy::localization::atlas
