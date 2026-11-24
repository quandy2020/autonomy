/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/coverage/coverage_evaluator.hpp"
#include <algorithm>

namespace autonomy::task::exploration {
CoverageRealization CoverageEvaluator::Apply(const perception::SensorObservation& o,
                                            double now, CoverageState& state) const {
    CoverageRealization result;
    if (o.map_epoch != state.epoch() || !o.Valid(now)) {
        result.reason = "invalid, stale or different-epoch observation"; return result;
    }
    std::map<CoverageId,double> evidence;
    for (const auto& entry : state.units()) {
        const auto& u = entry.second;
        double quality = 0;
        for (const auto& ray : o.rays) {
            const auto d = u.center-ray.origin;
            const double along = d.x*ray.direction.x+d.y*ray.direction.y+d.z*ray.direction.z;
            if (along < 0 || along > ray.distance+u.tolerance) continue;
            const double miss = (d-ray.direction*along).Norm();
            if (miss > u.tolerance) continue;
            // A frontier is resolved only by a measured surface at that cell.
            // A no-hit ray merely passes through unknown space; treating it as
            // evidence completed the visible map boundary without moving the
            // robot or making any of those cells known.
            if (!ray.hit || (ray.Endpoint()-u.center).Norm() > u.tolerance) continue;
            const double normal_norm = u.normal.Norm();
            double incidence = 1;
            if (normal_norm > 1e-6) {
                incidence = -(u.normal.x*ray.direction.x+u.normal.y*ray.direction.y+
                              u.normal.z*ray.direction.z)/normal_norm;
                if (incidence <= 0) continue;
            }
            quality = std::max(quality,ray.quality*incidence);
        }
        if (quality > 0) evidence[u.id] = quality;
    }
    for (const auto& p : evidence) {
        result.evidence.push_back({p.first,o.sensor,p.second});
        const double before = state.Remaining(p.first);
        if (state.Confirm(p.first,o.sensor,p.second,o.stamp)) {
            const auto& u = state.units().at(p.first);
            result.realized_gain += u.area*u.weight*(before-state.Remaining(p.first));
            ++result.confirmed_units;
        }
    }
    result.reason = evidence.empty() ? "no valid rays cover pending units" : "actual ray evidence applied";
    return result;
}
}  // namespace autonomy::task::exploration
