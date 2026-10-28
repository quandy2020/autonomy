#include "atlas/closure/pose_graph_optimizer.hpp"

#include "atlas/common/optimization/optimizer.hpp"

namespace autonomy::localization::atlas {

void PoseGraphOptimizer::Optimize(
    Map* map, KeyFrame* loop_keyframe, KeyFrame* current_keyframe,
    const LoopCloser::KeyFrameAndPose& non_corrected,
    const LoopCloser::KeyFrameAndPose& corrected,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& loop_connections,
    bool fix_scale) {
    Optimizer::OptimizeEssentialGraph(map, loop_keyframe, current_keyframe,
                                      non_corrected, corrected,
                                      loop_connections, fix_scale);
}

void PoseGraphOptimizer::OptimizeFourDof(
    Map* map, KeyFrame* loop_keyframe, KeyFrame* current_keyframe,
    const LoopCloser::KeyFrameAndPose& non_corrected,
    const LoopCloser::KeyFrameAndPose& corrected,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& loop_connections) {
    Optimizer::OptimizeEssentialGraph4DoF(map, loop_keyframe, current_keyframe,
                                          non_corrected, corrected,
                                          loop_connections);
}

}  // namespace autonomy::localization::atlas
