#pragma once

#include <map>
#include <set>
#include <vector>

#include "atlas/closure/loop_closer.hpp"

namespace autonomy::localization::atlas {

class KeyFrame;
class Map;
class MapPoint;

class PoseGraphOptimizer
{
public:
    static void Optimize(
        Map* map, KeyFrame* loop_keyframe, KeyFrame* current_keyframe,
        const LoopCloser::KeyFrameAndPose& non_corrected,
        const LoopCloser::KeyFrameAndPose& corrected,
        const std::map<KeyFrame*, std::set<KeyFrame*>>& loop_connections,
        bool fix_scale);

    static void OptimizeFourDof(
        Map* map, KeyFrame* loop_keyframe, KeyFrame* current_keyframe,
        const LoopCloser::KeyFrameAndPose& non_corrected,
        const LoopCloser::KeyFrameAndPose& corrected,
        const std::map<KeyFrame*, std::set<KeyFrame*>>& loop_connections);
};

}  // namespace autonomy::localization::atlas
