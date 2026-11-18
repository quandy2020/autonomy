/* Copyright 2026 The Openbot Authors. */
/** @file execution_esdf.hpp
 * @brief Immutable execution-frame distance evidence from real localized rays.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_validator.hpp"
#include "autonomy/perception/observation/observation_ingress.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

/** @brief Localization-owner attestation of a fixed world-to-execution alignment.
 * The owner must start a new alignment_epoch if this transform changes. All
 * acquisition times, now, valid_until and deadlines share clock_domain. This
 * value is additional evidence; map_epoch alone is not an odom alignment proof.
 */
struct ExecutionAlignment {
    perception::RigidTransform execution_from_world;  ///< Attested world-to-odom transform.
    std::string world_frame;       ///< Exact frame used by the measured rays.
    std::string execution_frame;   ///< Continuous frame used by trajectory references.
    std::string clock_domain;      ///< Explicit shared acquisition/execution clock identity.
    std::uint64_t map_epoch = 0;        ///< Matching observation world generation.
    std::uint64_t alignment_epoch = 0;  ///< Matching continuous reference alignment.
    double valid_from = 0.0;       ///< Earliest acquisition covered by this fixed transform.
    double valid_until = 0.0;      ///< Last execution time covered by this attestation.
    double position_error = 0.0;   ///< Conservative alignment error, meters.
    bool healthy = false;         ///< Explicit owner health, never inferred from frame names.
};

/** @brief One actual packet and its independently attested execution alignment. */
struct ExecutionObservation {
    perception::ObservationPacket packet;  ///< Complete acquisition-time pose proofs remain intact.
    ExecutionAlignment alignment;           ///< Exact frame/epoch/time conversion evidence.
};

/** @brief Fixed volume, source policy and finite work for one immutable rebuild. */
struct LocalEsdfOptions {
    perception::Vec3 origin{-3.2, -3.2, -1.0};  ///< Minimum execution-frame corner, meters.
    std::array<unsigned, 3> dimensions{64, 64, 32};  ///< XYZ voxel counts, each at most 128.
    double resolution = 0.1;               ///< Voxel side, meters; in [0.01, 1].
    perception::SensorMode mode = perception::SensorMode::kRgbdOnly;  ///< Required active sources.
    std::string execution_frame = "odom";  ///< Exact output and reference frame.
    std::string clock_domain;              ///< Caller-supplied shared clock identity.
    std::uint64_t revision = 0;            ///< Nonzero owner-assigned immutable snapshot identity.
    double maximum_observation_age = 0.5;  ///< Age bound applied to every actual ray, seconds.
    double surface_margin = 0.05;          ///< Stop carving this far before a hit, meters.
    double measurement_error = 0.02;       ///< Conservative endpoint/range error, meters.
    std::size_t maximum_input_elements = 500000;  ///< Total rays, proofs and depth pixels.
    std::size_t maximum_surface_samples = 262144;  ///< Retained real hits; overflow rejects publication.
    std::size_t maximum_work = 10000000;    ///< Carving/EDT/initialization work, at most 100 million.
    double deadline = 0.0;                 ///< Exclusive build deadline in clock_domain.
};

/** @brief Unknown is a blocking volume; free rays never erase a retained hit. */
enum class LocalVoxelState : std::uint8_t { kUnknown, kFree, kOccupied };

/** @brief One validated real hit, transformed by the packet's attested alignment.
 * Retaining a surface does not classify it as traversable ground. Acquisition
 * provenance is immutable and shares the exact occupancy snapshot lifetime.
 */
struct MeasuredSurfaceSample {
    perception::Vec3 endpoint;        ///< Measured execution-frame hit position, meters.
    perception::Vec3 sensor_origin;   ///< Actual execution-frame acquisition origin, meters.
    perception::SensorKind sensor;    ///< Actual source, never a synthesized virtual camera.
    std::uint64_t sequence = 0;       ///< Packet identity within that source.
    std::uint64_t calibration_revision = 0;  ///< Validated packet calibration identity.
    double stamp = 0.0;               ///< Actual per-beam acquisition time.
    double quality = 0.0;             ///< Measured geometric quality, not a probability.
};

struct LocalEsdfBuildResult;

/** @brief Owned, bounded and immutable occupancy plus an exact center-distance field.
 * Free ray traversal is a voxel-resolution sensing model, not proof that every
 * continuous point between beams was observed. Unknown cells and volume exterior
 * block execution. DistanceLowerBound subtracts voxel geometry and measurement/
 * alignment error; there is no optimistic trilinear interpolation.
 */
class LocalEsdfSnapshot {
public:
    /** @brief Caller-pinned build identity and volume; never modified after publication. */
    const LocalEsdfOptions& options() const { return options_; }
    /** @brief Continuous alignment required by every trajectory checked against this snapshot. */
    std::uint64_t alignment_epoch() const { return alignment_epoch_; }
    /** @brief Corresponding world alignment, independent of continuous execution alignment. */
    std::uint64_t map_epoch() const { return map_epoch_; }
    /** @brief Earliest input/pose/alignment/age expiry; rebuilding cannot renew old evidence. */
    double valid_until() const { return valid_until_; }
    /** @brief Publication time; a regressed query time cannot consume this snapshot. */
    double built_at() const { return built_at_; }
    /** @brief Maximum attested alignment error over all contributing observations. */
    double alignment_error() const { return alignment_error_; }
    /** @brief Bounded validated surface hits; consumers must retain this snapshot and check expiry. */
    const std::vector<MeasuredSurfaceSample>& surface_samples() const { return surface_samples_; }
    /** @brief Check voxel indices and freshness; exterior or invalid input yields unknown. */
    LocalVoxelState State(unsigned x, unsigned y, unsigned z, double now) const;
    /** @brief Cell-center position, in the continuous execution frame.
     * @throws std::out_of_range For an index outside the fixed volume.
     */
    perception::Vec3 Center(unsigned x, unsigned y, unsigned z) const;
    /** @brief Conservative distance to occupied/unknown voxels or volume exterior.
     * Returns zero for invalid, nonfree, expired, regressed or exterior queries.
     */
    double DistanceLowerBound(const perception::Vec3& point, double now) const;

private:
    /** @brief Only a successfully validated builder may create a usable snapshot. */
    LocalEsdfSnapshot() = default;
    friend struct LocalEsdfBuildResult;
    friend LocalEsdfBuildResult BuildLocalEsdf(const std::vector<ExecutionObservation>&,
                                              const LocalEsdfOptions&, const ExecutionClock&);
    LocalEsdfOptions options_;                    ///< Owned fixed build contract.
    std::uint64_t alignment_epoch_ = 0;           ///< Pinned odom generation.
    std::uint64_t map_epoch_ = 0;                 ///< Pinned world generation.
    double built_at_ = 0.0, valid_until_ = 0.0;    ///< Inclusive shared-clock support interval.
    double alignment_error_ = 0.0;               ///< Maximum attested position uncertainty.
    std::vector<LocalVoxelState> cells_;          ///< X-fast occupancy; retained occupied dominates free.
    std::vector<double> squared_distance_;       ///< Exact squared grid-unit distance to blocking centers.
    std::vector<MeasuredSurfaceSample> surface_samples_;  ///< Actual hits with unchanged source provenance.
};

/** @brief Build publishes nothing on invalid evidence, exceptions or exhausted work. */
struct LocalEsdfBuildResult {
    std::shared_ptr<const LocalEsdfSnapshot> snapshot;  ///< Nonnull only after complete validation/EDT.
    std::size_t work = 0;                    ///< Counted initialization, tracing and distance-transform work.
    std::size_t rays = 0;                    ///< Real rays transformed/traced, never synthesized pixels.
    std::string reason;                     ///< Explicit rejection cause; empty on success.
};

/** @brief Build from at most eight already decoded real observation packets.
 * All required active sources must be present and every packet must remain valid.
 * Each beam retains its own acquisition-time proof and age. The fixed alignment
 * must attest the whole acquisition interval, and all packets must agree on it.
 * Hits dominate free carving regardless of input order. Expiry leaves unknown
 * space on the next explicit rebuild; no stale value is silently refreshed.
 */
LocalEsdfBuildResult BuildLocalEsdf(const std::vector<ExecutionObservation>& observations,
                                  const LocalEsdfOptions& options, const ExecutionClock& clock);

}  // namespace autonomy::control::controller::policy_controller::sand_planner
