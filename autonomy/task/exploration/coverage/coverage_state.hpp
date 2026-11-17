/* Copyright 2026 The Openbot Authors. */
/** @file coverage_state.hpp
 * @brief Stable observation requirements and a ledger containing only measured evidence.
 */
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::task::exploration {
/** @brief Independently normalized observation categories. */
enum class CoverageKind { kFrontier, kSurface, kDirection };
/** @brief Stable spatial/directional identity, independent of raw point density. */
struct CoverageId {
    int x = 0;  ///< World-lattice X index.
    int y = 0;  ///< World-lattice Y index.
    int z = 0;  ///< World-lattice Z index.
    int direction = 0;  ///< Required viewing-direction bin.
    CoverageKind kind = CoverageKind::kFrontier;  ///< Observation category.
    /** @brief Deterministic lexicographic ordering for ledger keys. */
    bool operator<(const CoverageId& b) const {
        return std::tie(x,y,z,direction,kind) < std::tie(b.x,b.y,b.z,b.direction,b.kind);
    }
    /** @brief Checks whether two entries describe the same requirement. */
    bool operator==(const CoverageId& b) const { return !(*this<b) && !(b<*this); }
};
/** @brief One requirement whose evidence is populated only by actual observations. */
struct CoverageUnit {
    CoverageId id;  ///< Stable requirement identity.
    perception::Vec3 center;  ///< World-space target, in meters.
    perception::Vec3 normal;  ///< Surface normal; zero permits any incidence direction.
    double area = 1;  ///< Surface area or standardized coverage units.
    double weight = 1;  ///< Nonnegative task priority.
    double tolerance = 0.1;  ///< Ray matching tolerance, in meters.
    double required_quality = 0.8;  ///< Required raw quality in (0, 1].
    perception::SensorMask required_sensors = 0;  ///< Zero: either source; otherwise each set source.
    std::array<double, 2> observed_quality{0, 0};  ///< Raw measured maxima: LiDAR, RGB-D.
    std::array<double, 2> observed_stamp{0, 0};  ///< Accepted evidence times, in seconds.
    bool pending = true;  ///< Any required source is below its quality target.
};
/** @brief Raw quality contribution; neither normalized completion nor probability. */
struct CoverageContribution {
    CoverageId id;  ///< Requirement to which the contribution applies.
    perception::SensorKind sensor = perception::SensorKind::kRgbd;  ///< Observation source.
    double quality = 0;  ///< Raw quality in [0, 1], before task-target normalization.
};
/** @brief Measured coverage owned by the exploration session's update thread. */
class CoverageState {
public:
    /** @brief Clears requirements and evidence for a new alignment epoch. */
    void Reset(std::uint64_t epoch);
    /** @brief Updates a requirement without importing caller-supplied evidence.
     * @throws std::invalid_argument If identity, geometry or policy is invalid.
     * Measured evidence survives a policy change only when geometry is unchanged.
     */
    void Upsert(CoverageUnit unit);
    /** @brief Applies measured quality from a valid source and nondecreasing time.
     * @return True only when source quality increases; invalid or old evidence returns false.
     */
    bool Confirm(const CoverageId& id, perception::SensorKind sensor, double quality, double stamp);
    /** @brief Normalized quality residual, averaged over required sources. */
    double Remaining(const CoverageId& id) const;
    /** @brief Whether an existing requirement is fulfilled by measurements. */
    bool Complete(const CoverageId& id) const;
    /** @brief Explicitly removes a requirement; window movement must not imply removal. */
    void Remove(const CoverageId& id);
    /** @brief Clears evidence after its geometry is invalidated. */
    void Invalidate(const CoverageId& id);
    /** @brief Counts requirements awaiting measured quality evidence. */
    std::size_t PendingCount() const;
    /** @brief Read view valid until the owner's next mutation. */
    const std::map<CoverageId, CoverageUnit>& units() const { return units_; }
    /** @brief Alignment epoch of all requirements. */
    std::uint64_t epoch() const { return epoch_; }
    /** @brief Requirement/evidence revision, excluding predictions. */
    std::uint64_t revision() const { return revision_; }
private:
    std::map<CoverageId, CoverageUnit> units_;  ///< Requirements and actual evidence.
    std::uint64_t epoch_ = 0;  ///< Coordinate-alignment epoch.
    std::uint64_t revision_ = 0;  ///< Monotonic requirement/evidence revision.
};
}  // namespace autonomy::task::exploration
