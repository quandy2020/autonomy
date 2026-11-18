/* Copyright 2026 The Openbot Authors. */
/** @file coverage_fusion.cpp
 * @brief Measured coverage bookkeeping and normalized positive quality improvement.
 */
#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace autonomy::task::exploration {
namespace {
/** @brief Checks that a source has an explicitly supported ledger slot. */
bool SensorValid(perception::SensorKind sensor) {
    return sensor == perception::SensorKind::kLidar || sensor == perception::SensorKind::kRgbd;
}
/** @brief Checks that a category has an explicitly supported reference scale. */
bool KindValid(CoverageKind kind) {
    return kind == CoverageKind::kFrontier || kind == CoverageKind::kSurface ||
           kind == CoverageKind::kDirection;
}
/** @brief Validates raw quality without conflating it with completion. */
bool QualityValid(double quality) {
    return std::isfinite(quality) && quality >= 0 && quality <= 1;
}
/** @brief Maps a previously validated source to its quality-array slot. */
std::size_t SensorIndex(perception::SensorKind sensor) {
    return sensor == perception::SensorKind::kLidar ? 0 : 1;
}
/** @brief Normalizes raw quality to the task target, saturating at completion. */
double Completion(double quality, const CoverageUnit& unit) {
    return std::min(1.0, quality / unit.required_quality);
}
/** @brief Tests completion for either source or independently required sources. */
bool UnitComplete(const CoverageUnit& unit) {
    if (!unit.required_sensors)
        return std::max(unit.observed_quality[0], unit.observed_quality[1]) >= unit.required_quality;
    for (auto sensor : {perception::SensorKind::kLidar, perception::SensorKind::kRgbd})
        if ((unit.required_sensors & perception::Mask(sensor)) &&
            unit.observed_quality[SensorIndex(sensor)] < unit.required_quality) return false;
    return true;
}
/** @brief Validates every reservation before calculation or accumulation. */
void ValidateSelection(const CoverageSelection& selected) {
    for (const auto& item : selected.geometry)
        if (!KindValid(item.first.kind) || !QualityValid(item.second))
            throw std::invalid_argument("invalid predicted geometric reservation");
    for (const auto& item : selected.sources)
        if (!KindValid(item.first.first.kind) || !SensorValid(item.first.second) ||
            !QualityValid(item.second))
            throw std::invalid_argument("invalid predicted source reservation");
}
/** @brief Aggregated raw maxima retaining independent sources and their union. */
struct PredictedQuality {
    std::map<CoverageId, double> geometry;  ///< Any-source maximum per stable unit.
    std::map<CoverageId, std::array<double, 2>> sources;  ///< LiDAR/RGB-D maxima per unit.
};
/** @brief Validates and aggregates once, avoiding per-unit rescans of contributions. */
PredictedQuality Aggregate(const std::vector<CoverageContribution>& contributions) {
    PredictedQuality result;
    for (const auto& item : contributions) {
        if (!KindValid(item.id.kind) || !SensorValid(item.sensor) || !QualityValid(item.quality))
            throw std::invalid_argument("invalid coverage contribution");
        auto& geometry = result.geometry[item.id];
        geometry = std::max(geometry, item.quality);
        auto& quality = result.sources[item.id][SensorIndex(item.sensor)];
        quality = std::max(quality, item.quality);
    }
    return result;
}
}  // namespace

void CoverageState::Reset(std::uint64_t epoch) {
    units_.clear(); epoch_ = epoch; ++revision_; ++requirements_revision_;
}
void CoverageState::Upsert(CoverageUnit unit) {
    if (!KindValid(unit.id.kind) || !unit.center.Finite() || !unit.normal.Finite() ||
        !std::isfinite(unit.area) || unit.area <= 0 || !std::isfinite(unit.weight) || unit.weight < 0 ||
        !std::isfinite(unit.tolerance) || unit.tolerance <= 0 ||
        !std::isfinite(unit.required_quality) || unit.required_quality <= 0 ||
        unit.required_quality > 1 || unit.required_sensors > 3)
        throw std::invalid_argument("invalid coverage unit");
    const auto previous = units_.find(unit.id);
    if (previous != units_.end()) {
        const auto& old = previous->second;
        const bool same_geometry = unit.center.x == old.center.x && unit.center.y == old.center.y &&
            unit.center.z == old.center.z && unit.normal.x == old.normal.x &&
            unit.normal.y == old.normal.y && unit.normal.z == old.normal.z &&
            unit.tolerance == old.tolerance;
        if (same_geometry && unit.area == old.area && unit.weight == old.weight &&
            unit.required_quality == old.required_quality && unit.required_sensors == old.required_sensors)
            return;
        unit.observed_quality = same_geometry ? old.observed_quality : std::array<double, 2>{0, 0};
        unit.observed_stamp = same_geometry ? old.observed_stamp : std::array<double, 2>{0, 0};
    } else {
        unit.observed_quality = {0, 0}; unit.observed_stamp = {0, 0};
    }
    unit.pending = !UnitComplete(unit);
    units_[unit.id] = std::move(unit); ++revision_; ++requirements_revision_;
}
bool CoverageState::Confirm(const CoverageId& id, perception::SensorKind sensor,
                            double quality, double stamp) {
    const auto found = units_.find(id);
    if (found == units_.end() || !SensorValid(sensor) || !QualityValid(quality) || quality <= 0 ||
        !std::isfinite(stamp) || stamp < 0) return false;
    auto& unit = found->second;
    const auto index = SensorIndex(sensor);
    if (stamp < unit.observed_stamp[index]) return false;
    const double before = unit.observed_quality[index];
    unit.observed_quality[index] = std::max(before, quality);
    if (quality > before || quality >= unit.required_quality) unit.observed_stamp[index] = stamp;
    unit.pending = !UnitComplete(unit);
    if (unit.observed_quality[index] > before) { ++revision_; return true; }
    return false;
}
double CoverageState::Remaining(const CoverageId& id) const {
    const auto found = units_.find(id);
    if (found == units_.end() || !found->second.pending) return 0;
    const auto& unit = found->second;
    if (!unit.required_sensors)
        return 1 - Completion(std::max(unit.observed_quality[0], unit.observed_quality[1]), unit);
    double completed = 0; unsigned count = 0;
    for (auto sensor : {perception::SensorKind::kLidar, perception::SensorKind::kRgbd})
        if (unit.required_sensors & perception::Mask(sensor)) {
            completed += Completion(unit.observed_quality[SensorIndex(sensor)], unit); ++count;
        }
    return 1 - completed / count;
}
bool CoverageState::Complete(const CoverageId& id) const {
    const auto found = units_.find(id); return found != units_.end() && !found->second.pending;
}
void CoverageState::Remove(const CoverageId& id) {
    if (units_.erase(id)) { ++revision_; ++requirements_revision_; }
}
void CoverageState::Invalidate(const CoverageId& id) {
    const auto found = units_.find(id); if (found == units_.end()) return;
    found->second.observed_quality = {0, 0}; found->second.observed_stamp = {0, 0};
    found->second.pending = true; ++revision_; ++requirements_revision_;
}
std::size_t CoverageState::PendingCount() const {
    return std::count_if(units_.begin(), units_.end(), [](const auto& item) { return item.second.pending; });
}
void CoverageState::InvalidateSources(perception::SensorMask sources) {
    if (!sources || sources > 3) throw std::invalid_argument("invalid lost observation sources");
    for (auto& entry : units_) {
        auto& unit = entry.second;
        for (unsigned i = 0; i < 2; ++i)
            if (sources & (1U << i)) {
                unit.observed_quality[i] = 0;
                unit.observed_stamp[i] = 0;
            }
        unit.pending = !UnitComplete(unit);
    }
    ++revision_; ++requirements_revision_;
}
std::map<CoverageId, double> CoverageFusion::Fuse(const std::vector<CoverageContribution>& contributions) {
    return Aggregate(contributions).geometry;
}
double CoverageFusion::Gain(const CoverageState& state,
    const std::vector<CoverageContribution>& contributions, const std::map<CoverageId, double>& selected,
    const std::array<double, 3>& scales) {
    CoverageSelection selection; selection.geometry = selected;
    return MarginalGain(state, contributions, selection, scales);
}
void CoverageFusion::Accumulate(const std::vector<CoverageContribution>& contributions,
                                CoverageSelection& selected) {
    ValidateSelection(selected);
    const auto predicted = Aggregate(contributions);
    for (const auto& item : predicted.geometry) {
        auto& quality = selected.geometry[item.first]; quality = std::max(quality, item.second);
    }
    for (const auto& item : predicted.sources)
        for (auto sensor : {perception::SensorKind::kLidar, perception::SensorKind::kRgbd}) {
            const double quality = item.second[SensorIndex(sensor)];
            if (quality <= 0) continue;
            auto& reservation = selected.sources[{item.first, sensor}];
            reservation = std::max(reservation, quality);
        }
}
double CoverageFusion::MarginalGain(const CoverageState& state,
    const std::vector<CoverageContribution>& contributions, const CoverageSelection& selected,
    const std::array<double, 3>& scales) {
    for (double scale : scales)
        if (!std::isfinite(scale) || scale <= 0)
            throw std::invalid_argument("coverage reference scale must be positive");
    ValidateSelection(selected);
    const auto predicted = Aggregate(contributions);
    double gain = 0;
    for (const auto& item : predicted.geometry) {
        const auto found = state.units().find(item.first);
        if (found == state.units().end()) continue;
        const auto& unit = found->second;
        double improvement = 0;
        if (!unit.required_sensors) {
            const auto reserved = selected.geometry.find(item.first);
            const double baseline = std::max({unit.observed_quality[0], unit.observed_quality[1],
                reserved == selected.geometry.end() ? 0.0 : reserved->second});
            improvement = std::max(0.0, Completion(item.second, unit) - Completion(baseline, unit));
        } else {
            unsigned count = 0;
            for (auto sensor : {perception::SensorKind::kLidar, perception::SensorKind::kRgbd}) {
                if (!(unit.required_sensors & perception::Mask(sensor))) continue;
                ++count;
                const auto reserved = selected.sources.find({item.first, sensor});
                const auto index = SensorIndex(sensor);
                const double baseline = std::max(unit.observed_quality[index],
                    reserved == selected.sources.end() ? 0.0 : reserved->second);
                improvement += std::max(0.0,
                    Completion(predicted.sources.at(item.first)[index], unit) - Completion(baseline, unit));
            }
            improvement /= count;
        }
        gain += unit.area * unit.weight * improvement / scales[static_cast<std::size_t>(unit.id.kind)];
    }
    return gain;
}
}  // namespace autonomy::task::exploration
