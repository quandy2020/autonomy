/*
 * Copyright 2026 The Openbot Authors
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_LOCAL_MAP_TRACKING_TRANSACTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_LOCAL_MAP_TRACKING_TRANSACTION_HPP_

#include <cstddef>
#include <tuple>
#include <unordered_map>

#include "atlas/sensor/keyframe.hpp"
#include "atlas/sensor/map_point.hpp"

namespace autonomy::localization::atlas::internal {

using MapPointTrackingValues =
    std::tuple<float, float, float, float, float, float, bool, bool, int, int,
               float, float, unsigned long, unsigned long>;

inline MapPointTrackingValues MapPointTrackingSnapshot(MapPoint* point) {
    return {point->mTrackProjX,
            point->mTrackProjY,
            point->mTrackDepth,
            point->mTrackDepthR,
            point->mTrackProjXR,
            point->mTrackProjYR,
            point->mbTrackInView,
            point->mbTrackInViewR,
            point->mnTrackScaleLevel,
            point->mnTrackScaleLevelR,
            point->mTrackViewCos,
            point->mTrackViewCosR,
            point->mnTrackReferenceForFrame,
            point->mnLastFrameSeen};
}

inline void RestoreMapPointTracking(MapPoint* point,
                                    const MapPointTrackingValues& values) {
    std::tie(point->mTrackProjX, point->mTrackProjY, point->mTrackDepth,
             point->mTrackDepthR, point->mTrackProjXR, point->mTrackProjYR,
             point->mbTrackInView, point->mbTrackInViewR,
             point->mnTrackScaleLevel, point->mnTrackScaleLevelR,
             point->mTrackViewCos, point->mTrackViewCosR,
             point->mnTrackReferenceForFrame, point->mnLastFrameSeen) = values;
}

class LocalMapTrackingTransaction
{
public:
    LocalMapTrackingTransaction() = default;
    void Reserve(std::size_t expected_points,
                 std::size_t expected_keyframes = 0) {
        point_states_.reserve(expected_points);
        keyframe_states_.reserve(expected_keyframes);
    }
    ~LocalMapTrackingTransaction() {
        Restore();
    }
    LocalMapTrackingTransaction(const LocalMapTrackingTransaction&) = delete;
    LocalMapTrackingTransaction& operator=(const LocalMapTrackingTransaction&) =
        delete;

    void Capture(MapPoint* point) {
        if (point) {
            point_states_.try_emplace(point, MapPointTrackingSnapshot(point));
        }
    }
    void Capture(KeyFrame* keyframe) {
        if (keyframe) {
            keyframe_states_.try_emplace(keyframe,
                                         keyframe->mnTrackReferenceForFrame);
        }
    }

    void Restore() {
        if (!active_) {
            return;
        }
        for (const auto& [point, values] : point_states_) {
            RestoreMapPointTracking(point, values);
        }
        for (const auto& [keyframe, marker] : keyframe_states_) {
            keyframe->mnTrackReferenceForFrame = marker;
        }
        active_ = false;
    }
    void Commit() {
        active_ = false;
    }

private:
    std::unordered_map<MapPoint*, MapPointTrackingValues> point_states_;
    std::unordered_map<KeyFrame*, unsigned long> keyframe_states_;
    bool active_ = true;
};

}  // namespace autonomy::localization::atlas::internal
#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_LOCAL_MAP_TRACKING_TRANSACTION_HPP_
