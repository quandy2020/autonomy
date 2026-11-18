/* Copyright 2026 The Openbot Authors. */
/** @file execution_esdf.cpp
 * @brief Bounded real-ray carving and separable exact Euclidean distance transform.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/execution_esdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief Count finite work and periodically check an authoritative monotonic clock. */
class WorkBudget {
public:
    /** @brief Capture a fixed deadline and initial clock; callbacks may not block indefinitely. */
    WorkBudget(std::size_t maximum, double deadline, ExecutionClock clock, double now)
        : maximum_(maximum), deadline_(deadline), clock_(std::move(clock)), last_(now) {}
    /** @brief Every loop consumes work; check time at least every 64 units. */
    void Spend() {
        if (used_ == maximum_) throw std::runtime_error("local ESDF work budget exhausted");
        ++used_;
        if ((used_ & 63U) == 0) Check();
    }
    /** @brief Reject nonfinite/regressed time or completion at/after the deadline. */
    double Check() {
        const double now = clock_();
        if (!std::isfinite(now) || now < last_ || now >= deadline_)
            throw std::runtime_error("local ESDF clock regressed or deadline expired");
        last_ = now;
        return now;
    }
    /** @brief Actual counted work, including work preceding a rejected publication. */
    std::size_t used() const { return used_; }
private:
    std::size_t maximum_;  ///< Hard work count, checked before increment.
    double deadline_;      ///< Exclusive shared-clock completion deadline.
    ExecutionClock clock_; ///< Authoritative bounded clock dependency.
    double last_;          ///< Last successfully checked time.
    std::size_t used_ = 0;  ///< All traced/initialized/transformed work.
};

/** @brief X-fast storage index; dimensions were already bounded and validated. */
std::size_t Index(const LocalEsdfOptions& o, unsigned x, unsigned y, unsigned z) {
    return (static_cast<std::size_t>(z) * o.dimensions[1] + y) * o.dimensions[0] + x;
}
/** @brief Read one XYZ component without relying on structure layout. */
std::array<double, 3> Components(const perception::Vec3& v) { return {v.x, v.y, v.z}; }
/** @brief Fixed alignments in one epoch must represent the same rigid transform. */
bool SameTransform(const perception::RigidTransform& a, const perception::RigidTransform& b) {
    if ((a.translation - b.translation).Norm() > 1e-9) return false;
    for (std::size_t i = 0; i < a.rotation.size(); ++i)
        if (std::abs(a.rotation[i] - b.rotation[i]) > 1e-9) return false;
    return true;
}

/** @brief Shared acquisition times must attest the same base pose across sources. */
void CheckBaseAgreement(const perception::ObservationPacket& a, const perception::ObservationPacket& b,
                        WorkBudget& budget) {
    const auto check = [&](const perception::AcquisitionLocalization& x,
                           const perception::AcquisitionLocalization& y) {
        budget.Spend();
        if (!SameTransform(x.world_from_base, y.world_from_base))
            throw std::invalid_argument("conflicting base poses at one acquisition time");
    };
    if (a.observation.stamp == b.observation.stamp) check(a.localization, b.localization);
    const auto match_end = [&](const perception::ObservationPacket& end,
                               const perception::ObservationPacket& samples) {
        const auto found = std::lower_bound(samples.acquisition_poses.begin(), samples.acquisition_poses.end(),
            end.observation.stamp, [](const auto& sample, double stamp) { return sample.stamp < stamp; });
        if (found != samples.acquisition_poses.end() && found->stamp == end.observation.stamp)
            check(end.localization, found->localization);
    };
    match_end(a, b); match_end(b, a);
    std::size_t i = 0, j = 0;
    while (i < a.acquisition_poses.size() && j < b.acquisition_poses.size()) {
        budget.Spend();
        const auto& x = a.acquisition_poses[i]; const auto& y = b.acquisition_poses[j];
        if (x.stamp == y.stamp) { check(x.localization, y.localization); ++i; ++j; }
        else if (x.stamp < y.stamp) ++i;
        else ++j;
    }
}

/** @brief Clip an actual free ray interval and visit cells with positive interior traversal.
 * Simultaneous boundary crossings advance all tied axes; touching a corner or
 * plane does not carve adjacent cells. Only retained measured free segments are
 * traversed. A hit cell already present is never cleared by another source.
 */
void Carve(const LocalEsdfOptions& o, const perception::Vec3& origin,
           const perception::Vec3& direction, double range,
           std::vector<LocalVoxelState>& cells, WorkBudget& budget) {
    if (range <= 0.0) return;
    const auto p = Components(origin), d = Components(direction), lo = Components(o.origin);
    double first = 0.0, last = range;
    for (unsigned a = 0; a < 3; ++a) {
        const double hi = lo[a] + o.dimensions[a] * o.resolution;
        if (d[a] == 0.0) {
            if (p[a] <= lo[a] || p[a] >= hi) return;
        } else {
            const double t0 = (lo[a] - p[a]) / d[a], t1 = (hi - p[a]) / d[a];
            first = std::max(first, std::min(t0, t1));
            last = std::min(last, std::max(t0, t1));
        }
    }
    if (!(first < last) || !std::isfinite(first) || !std::isfinite(last)) return;
    std::array<int, 3> cell{};
    std::array<double, 3> next{}, delta{};
    for (unsigned a = 0; a < 3; ++a) {
        const double start = p[a] + d[a] * first;
        double grid = (start - lo[a]) / o.resolution;
        grid = std::nextafter(grid, d[a] < 0.0 ? -std::numeric_limits<double>::infinity() :
                                                               std::numeric_limits<double>::infinity());
        cell[a] = static_cast<int>(std::floor(std::clamp(grid, 0.0, static_cast<double>(o.dimensions[a] - 1))));
        if (d[a] == 0.0) {
            next[a] = delta[a] = std::numeric_limits<double>::infinity();
        } else {
            const double boundary = lo[a] + (cell[a] + (d[a] > 0.0 ? 1 : 0)) * o.resolution;
            next[a] = first + (boundary - start) / d[a];
            delta[a] = o.resolution / std::abs(d[a]);
        }
    }
    double current = first;
    while (current < last) {
        budget.Spend();
        if (cell[0] < 0 || cell[1] < 0 || cell[2] < 0 ||
            cell[0] >= static_cast<int>(o.dimensions[0]) ||
            cell[1] >= static_cast<int>(o.dimensions[1]) ||
            cell[2] >= static_cast<int>(o.dimensions[2])) break;
        const double boundary = std::min({next[0], next[1], next[2], last});
        if (boundary > current + 1e-12 * o.resolution) {
            auto& state = cells[Index(o, cell[0], cell[1], cell[2])];
            if (state != LocalVoxelState::kOccupied) state = LocalVoxelState::kFree;
        }
        if (boundary >= last) break;
        for (unsigned a = 0; a < 3; ++a) {
            if (next[a] <= boundary + 1e-12 * o.resolution) {
                cell[a] += d[a] > 0.0 ? 1 : -1;
                next[a] += delta[a];
            }
        }
        if (boundary < current) throw std::runtime_error("invalid ray traversal ordering");
        current = boundary;
    }
}

/** @brief Mark every voxel intersecting the measured hit's uncertainty ball. */
void MarkHit(const LocalEsdfOptions& o, const perception::Vec3& endpoint,
             std::vector<LocalVoxelState>& cells, WorkBudget& budget) {
    const auto p = Components(endpoint), lo = Components(o.origin);
    std::array<unsigned, 3> begin{}, end{};
    for (unsigned a = 0; a < 3; ++a) {
        const double hi = lo[a] + o.dimensions[a] * o.resolution;
        if (p[a] + o.measurement_error < lo[a] || p[a] - o.measurement_error >= hi) return;
        const double left = std::floor((p[a] - o.measurement_error - lo[a]) / o.resolution);
        const double right = std::floor((p[a] + o.measurement_error - lo[a]) / o.resolution);
        begin[a] = static_cast<unsigned>(std::clamp(left, 0.0, static_cast<double>(o.dimensions[a] - 1)));
        end[a] = static_cast<unsigned>(std::clamp(right, 0.0, static_cast<double>(o.dimensions[a] - 1)));
    }
    for (unsigned z = begin[2]; z <= end[2]; ++z)
        for (unsigned y = begin[1]; y <= end[1]; ++y)
            for (unsigned x = begin[0]; x <= end[0]; ++x) {
                budget.Spend();
                const std::array<unsigned, 3> c{x, y, z};
                double squared = 0.0;
                for (unsigned a = 0; a < 3; ++a) {
                    const double lower = lo[a] + c[a] * o.resolution;
                    const double gap = std::max({lower - p[a], p[a] - lower - o.resolution, 0.0});
                    squared += gap * gap;
                }
                if (squared <= o.measurement_error * o.measurement_error + 1e-15)
                    cells[Index(o, x, y, z)] = LocalVoxelState::kOccupied;
            }
}

/** @brief Exact one-dimensional min_j(f[j] + (i-j)^2), with infinite absent sites.
 * Maintains the lower envelope of parabolas. Work includes discarded envelope
 * sites, so degenerate input cannot evade the fixed bound.
 */
void TransformLine(const std::vector<double>& f, std::vector<double>& output, WorkBudget& budget) {
    std::vector<unsigned> sites(f.size());
    std::vector<double> breaks(f.size() + 1);
    int top = -1;
    for (unsigned q = 0; q < f.size(); ++q) {
        budget.Spend();
        if (!std::isfinite(f[q])) continue;
        double s = -std::numeric_limits<double>::infinity();
        while (top >= 0) {
            const double v = sites[top];
            s = ((f[q] + static_cast<double>(q) * q) - (f[sites[top]] + v * v)) / (2.0 * (q - v));
            if (s > breaks[top]) break;
            --top;
            budget.Spend();
        }
        ++top;
        sites[top] = q;
        breaks[top] = top == 0 ? -std::numeric_limits<double>::infinity() : s;
        breaks[top + 1] = std::numeric_limits<double>::infinity();
    }
    unsigned envelope = 0;
    for (unsigned q = 0; q < f.size(); ++q) {
        budget.Spend();
        if (top < 0) { output[q] = std::numeric_limits<double>::infinity(); continue; }
        while (breaks[envelope + 1] < q) { ++envelope; budget.Spend(); }
        const double d = static_cast<double>(q) - sites[envelope];
        output[q] = f[sites[envelope]] + d * d;
    }
}

/** @brief Three separable transforms give exact squared Euclidean center distances. */
void DistanceTransform(const LocalEsdfOptions& o, std::vector<double>& field, WorkBudget& budget) {
    for (unsigned axis = 0; axis < 3; ++axis) {
        const unsigned a = (axis + 1) % 3, b = (axis + 2) % 3;
        std::vector<double> line(o.dimensions[axis]), output(line.size());
        for (unsigned i = 0; i < o.dimensions[a]; ++i)
            for (unsigned j = 0; j < o.dimensions[b]; ++j) {
                std::array<unsigned, 3> c{};
                c[a] = i; c[b] = j;
                for (unsigned q = 0; q < line.size(); ++q) {
                    budget.Spend(); c[axis] = q; line[q] = field[Index(o, c[0], c[1], c[2])];
                }
                TransformLine(line, output, budget);
                for (unsigned q = 0; q < line.size(); ++q) {
                    budget.Spend(); c[axis] = q; field[Index(o, c[0], c[1], c[2])] = output[q];
                }
            }
    }
}
}  // namespace

LocalVoxelState LocalEsdfSnapshot::State(unsigned x, unsigned y, unsigned z, double now) const {
    if (!std::isfinite(now) || now < built_at_ || now > valid_until_ ||
        x >= options_.dimensions[0] || y >= options_.dimensions[1] || z >= options_.dimensions[2])
        return LocalVoxelState::kUnknown;
    return cells_[Index(options_, x, y, z)];
}

perception::Vec3 LocalEsdfSnapshot::Center(unsigned x, unsigned y, unsigned z) const {
    if (x >= options_.dimensions[0] || y >= options_.dimensions[1] || z >= options_.dimensions[2])
        throw std::out_of_range("local ESDF cell outside volume");
    return options_.origin + perception::Vec3{(x + 0.5) * options_.resolution,
        (y + 0.5) * options_.resolution, (z + 0.5) * options_.resolution};
}

double LocalEsdfSnapshot::DistanceLowerBound(const perception::Vec3& p, double now) const {
    if (!p.Finite() || !std::isfinite(now) || now < built_at_ || now > valid_until_) return 0.0;
    const auto point = Components(p), lo = Components(options_.origin);
    std::array<unsigned, 3> c{};
    double boundary = std::numeric_limits<double>::infinity();
    for (unsigned a = 0; a < 3; ++a) {
        const double hi = lo[a] + options_.dimensions[a] * options_.resolution;
        if (point[a] <= lo[a] || point[a] >= hi) return 0.0;
        c[a] = static_cast<unsigned>(std::floor((point[a] - lo[a]) / options_.resolution));
        boundary = std::min({boundary, point[a] - lo[a], hi - point[a]});
    }
    if (State(c[0], c[1], c[2], now) != LocalVoxelState::kFree) return 0.0;
    const double center_distance = std::sqrt(squared_distance_[Index(options_, c[0], c[1], c[2])]) * options_.resolution;
    const double lower = center_distance - std::sqrt(3.0) * options_.resolution / 2.0 -
                         (p - Center(c[0], c[1], c[2])).Norm();
    return std::max(0.0, std::min(lower, boundary) - options_.measurement_error - alignment_error_);
}

LocalEsdfBuildResult BuildLocalEsdf(const std::vector<ExecutionObservation>& observations,
                                  const LocalEsdfOptions& o, const ExecutionClock& clock) {
    LocalEsdfBuildResult result;
    std::unique_ptr<WorkBudget> budget;
    try {
        if (!clock || observations.empty() || observations.size() > 8 || !o.origin.Finite() ||
            std::abs(o.origin.x) > 1e6 || std::abs(o.origin.y) > 1e6 || std::abs(o.origin.z) > 1e6 ||
            !std::isfinite(o.resolution) || o.resolution < 0.01 || o.resolution > 1.0 ||
            (o.mode != perception::SensorMode::kLidarOnly && o.mode != perception::SensorMode::kRgbdOnly &&
             o.mode != perception::SensorMode::kLidarRgbd) || o.execution_frame.empty() || o.clock_domain.empty() ||
            !o.revision || !std::isfinite(o.maximum_observation_age) || o.maximum_observation_age <= 0.0 ||
            !std::isfinite(o.surface_margin) || o.surface_margin < 0.0 || o.surface_margin > 1.0 ||
            !std::isfinite(o.measurement_error) || o.measurement_error < 0.0 || o.measurement_error > 1.0 ||
            !o.maximum_input_elements || o.maximum_input_elements > 2000000 ||
            !o.maximum_surface_samples || o.maximum_surface_samples > 500000 ||
            !o.maximum_work || o.maximum_work > 100000000 || !std::isfinite(o.deadline))
            throw std::invalid_argument("invalid local ESDF volume, policy or budget");
        std::size_t volume = 1;
        for (unsigned count : o.dimensions) {
            if (!count || count > 128) throw std::invalid_argument("invalid local ESDF axis size");
            volume *= count;
        }
        if (volume > 262144) throw std::invalid_argument("local ESDF volume exceeds memory bound");
        std::size_t elements = 0;
        const auto add_elements = [&](std::size_t count) {
            if (count > o.maximum_input_elements - elements)
                throw std::invalid_argument("local ESDF input exceeds element bound");
            elements += count;
        };
        for (const auto& input : observations) {
            add_elements(input.packet.observation.rays.size());
            add_elements(input.packet.acquisition_poses.size());
            if (input.packet.depth_view) {
                add_elements(input.packet.depth_view->axial_depth.size());
                add_elements(input.packet.depth_view->valid.size());
            }
        }
        const double initial = clock();
        if (!std::isfinite(initial) || initial < 0.0 || initial >= o.deadline)
            throw std::invalid_argument("expired local ESDF build deadline or invalid time");
        budget = std::make_unique<WorkBudget>(o.maximum_work, o.deadline, clock, initial);
        auto snapshot = std::shared_ptr<LocalEsdfSnapshot>(new LocalEsdfSnapshot());
        snapshot->options_ = o;
        snapshot->valid_until_ = std::numeric_limits<double>::infinity();
        snapshot->cells_.resize(volume, LocalVoxelState::kUnknown);
        snapshot->squared_distance_.resize(volume);
        for (std::size_t i = 0; i < volume; ++i) budget->Spend();
        perception::SensorMask sources = 0;
        const auto& first = observations.front().alignment;
        for (std::size_t input_index = 0; input_index < observations.size(); ++input_index) {
            const auto& input = observations[input_index];
            const auto& packet = input.packet;
            const auto& observation = packet.observation;
            const auto& alignment = input.alignment;
            budget->Check();
            // Charge a conservative validation bound before calling the shared
            // packet validator. Its binary-search beam proofs and rigid-pose/
            // pixel checks remain bounded by maximum_input_elements. This
            // synchronous function is not forcibly preempted mid-instruction.
            unsigned proof_search = 1;
            for (auto size = packet.acquisition_poses.size(); size > 1; size >>= 1) ++proof_search;
            for (std::size_t i = 0; i < packet.observation.rays.size(); ++i)
                for (unsigned step = 0; step < proof_search + 2; ++step) budget->Spend();
            for (std::size_t i = 0; i < packet.acquisition_poses.size(); ++i) budget->Spend();
            if (packet.depth_view)
                for (std::size_t i = 0; i < packet.depth_view->axial_depth.size(); ++i) budget->Spend();
            if (!packet.Valid(initial) || !(perception::Mask(o.mode) & perception::Mask(observation.sensor)) ||
                !alignment.healthy || !alignment.execution_from_world.Valid() ||
                alignment.world_frame != observation.frame_id || alignment.execution_frame != o.execution_frame ||
                alignment.clock_domain != o.clock_domain || alignment.map_epoch != observation.map_epoch ||
                !alignment.alignment_epoch || !std::isfinite(alignment.valid_from) ||
                !std::isfinite(alignment.valid_until) || alignment.valid_from > observation.stamp ||
                alignment.valid_until < initial || !std::isfinite(alignment.position_error) ||
                alignment.position_error < 0.0 || alignment.position_error > 1.0 ||
                alignment.map_epoch != first.map_epoch || alignment.alignment_epoch != first.alignment_epoch ||
                alignment.world_frame != first.world_frame ||
                !SameTransform(alignment.execution_from_world, first.execution_from_world))
                throw std::invalid_argument("invalid packet or incompatible execution alignment");
            budget->Check();
            for (std::size_t previous = 0; previous < input_index; ++previous) {
                const auto& earlier = observations[previous].packet;
                budget->Spend();
                if (earlier.observation.sensor == observation.sensor) {
                    if (earlier.observation.sequence == observation.sequence ||
                        earlier.localization.calibration_revision != packet.localization.calibration_revision ||
                        (earlier.observation.sequence < observation.sequence && earlier.observation.stamp > observation.stamp) ||
                        (earlier.observation.sequence > observation.sequence && earlier.observation.stamp < observation.stamp))
                        throw std::invalid_argument("duplicate source identity, calibration change or time-order conflict");
                }
                CheckBaseAgreement(earlier, packet, *budget);
            }
            sources |= perception::Mask(observation.sensor);
            snapshot->alignment_error_ = std::max(snapshot->alignment_error_, alignment.position_error);
            snapshot->valid_until_ = std::min({snapshot->valid_until_, observation.valid_until,
                                              packet.localization.valid_until, alignment.valid_until});
            for (const auto& ray : observation.rays) {
                budget->Spend();
                if (ray.stamp < alignment.valid_from || ray.stamp > alignment.valid_until ||
                    initial > ray.stamp + o.maximum_observation_age)
                    throw std::invalid_argument("beam outside alignment support or observation age");
                snapshot->valid_until_ = std::min(snapshot->valid_until_, ray.stamp + o.maximum_observation_age);
                const auto origin = alignment.execution_from_world.Apply(ray.origin);
                const auto direction = alignment.execution_from_world.Rotate(ray.direction);
                const auto endpoint = origin + direction * ray.distance;
                if (!origin.Finite() || !direction.Finite() || !endpoint.Finite())
                    throw std::invalid_argument("ray conversion overflow");
                Carve(o, origin, direction, ray.distance - (ray.hit ? o.surface_margin + o.measurement_error : 0.0),
                      snapshot->cells_, *budget);
                if (ray.hit) {
                    budget->Spend();
                    if (snapshot->surface_samples_.size() == o.maximum_surface_samples)
                        throw std::runtime_error("local ESDF surface sample memory bound exhausted");
                    snapshot->surface_samples_.push_back({endpoint, origin, observation.sensor,
                        observation.sequence, packet.localization.calibration_revision, ray.stamp, ray.quality});
                    MarkHit(o, endpoint, snapshot->cells_, *budget);
                }
                ++result.rays;
            }
        }
        if (sources != perception::Mask(o.mode)) throw std::invalid_argument("required active source is absent");
        snapshot->alignment_epoch_ = first.alignment_epoch;
        snapshot->map_epoch_ = first.map_epoch;
        for (std::size_t i = 0; i < volume; ++i) {
            budget->Spend();
            snapshot->squared_distance_[i] = snapshot->cells_[i] == LocalVoxelState::kFree ?
                std::numeric_limits<double>::infinity() : 0.0;
        }
        DistanceTransform(o, snapshot->squared_distance_, *budget);
        snapshot->built_at_ = budget->Check();
        if (snapshot->built_at_ > snapshot->valid_until_)
            throw std::runtime_error("local ESDF evidence expired during construction");
        result.snapshot = std::move(snapshot);
        result.work = budget->used();
    } catch (const std::exception& exception) {
        result.reason = exception.what();
        if (budget) result.work = budget->used();
    } catch (...) {
        result.reason = "nonstandard exception building local ESDF";
        if (budget) result.work = budget->used();
    }
    return result;
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
