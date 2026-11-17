/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/coverage/local_coverage_planner.hpp"
#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"
#include <algorithm>
#include <set>
namespace autonomy::task::exploration {
std::vector<Viewpoint> LocalCoveragePlanner::Select(const std::vector<Viewpoint>& candidates,
    const CoverageState& state,std::size_t maximum,const std::optional<CoverageId>& mandatory) const {
    std::vector<Viewpoint> selected;CoverageSelection covered;std::set<std::uint64_t> used;
    while(selected.size()<maximum) {
        const Viewpoint* best=nullptr;double best_utility=-1;
        for(const auto& candidate:candidates) {
            if(used.count(candidate.id))continue;
            if(selected.empty() && mandatory && !std::any_of(candidate.contributions.begin(),candidate.contributions.end(),
               [&](const auto& c){return c.id==*mandatory && c.quality>0;}))continue;
            double gain=CoverageFusion::MarginalGain(state,candidate.contributions,covered);
            if (gain <= 0) {
                continue;
            }
            const double utility = candidate.Utility(gain);
            if(utility>best_utility) {best=&candidate;best_utility=utility;}
        }
        if(!best)break;
        auto view=*best;view.predicted_gain=CoverageFusion::MarginalGain(state,view.contributions,covered);
        selected.push_back(std::move(view));used.insert(best->id);
        CoverageFusion::Accumulate(best->contributions,covered);
    }
    return selected;
}
}  // namespace autonomy::task::exploration
