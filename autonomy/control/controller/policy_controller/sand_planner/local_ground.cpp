/* Copyright 2026 The Openbot Authors. */
/** @file local_ground.cpp
 * @brief Measured cell-plane reconstruction and bounded swept support admission.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/local_ground.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief One finite clock/work scope; every sample and examined cell consumes work. */
class GroundBudget {
public:
    /** @brief Capture a shared-clock deadline and independently bounded clock dependency. */
    GroundBudget(std::size_t maximum, double deadline, const ExecutionClock& clock, double initial)
        : maximum_(maximum), deadline_(deadline), clock_(clock), last_(initial) {}
    /** @brief Reject before increment; poll time at least every 64 units. */
    void Spend() {
        if (used_ == maximum_) throw std::runtime_error("measured ground work budget exhausted");
        ++used_;
        if ((used_ & 63U) == 0) Check();
    }
    /** @brief Time regression, nonfinite time or an expired deadline denies publication/admission. */
    double Check() {
        const double now = clock_();
        if (!std::isfinite(now) || now < last_ || now >= deadline_)
            throw std::runtime_error("measured ground clock regressed or deadline expired");
        last_ = now;
        return now;
    }
    /** @brief Actual work including attempts that were rejected. */
    std::size_t used() const { return used_; }
private:
    std::size_t maximum_;           ///< Fixed complete-operation work bound.
    double deadline_;               ///< Exclusive shared-clock deadline.
    const ExecutionClock& clock_;   ///< Caller-owned authoritative clock.
    double last_;                   ///< Last successfully checked time.
    std::size_t used_ = 0;           ///< Actual examined/fitted/verified elements.
};

/** @brief Require finite positive numeric configuration. */
bool Positive(double value) { return std::isfinite(value) && value > 0.0; }
/** @brief X-fast XY storage; all dimensions and indices have already been bounded. */
std::size_t Index(const LocalEsdfOptions& geometry, unsigned x, unsigned y) {
    return static_cast<std::size_t>(y) * geometry.dimensions[0] + x;
}
/** @brief Measured patch prediction, relative to the explicitly supplied cell center. */
double Height(const GroundPatch& patch, double dx, double dy) {
    return patch.center_height + patch.slope_x * dx + patch.slope_y * dy;
}

/** @brief Fit centered XY statistics and check spatial distribution plus every residual.
 * All selected hits in a column contribute; contradictory heights cannot be
 * hidden by choosing a convenient lower surface. This is deliberately conservative
 * near vertical obstacles and overhangs, which also remain occupied in the ESDF.
 */
GroundPatch Fit(const std::vector<const MeasuredSurfaceSample*>& hits, double cx, double cy,
                double resolution, double endpoint_error, const LocalGroundOptions& options,
                GroundBudget& budget) {
    GroundPatch patch;
    patch.samples = hits.size();
    if (hits.size() < options.minimum_samples) return patch;
    double sx = 0, sy = 0, sz = 0;
    unsigned quadrants = 0;
    for (const auto* hit : hits) {
        budget.Spend();
        const double x = hit->endpoint.x - cx, y = hit->endpoint.y - cy;
        sx += x; sy += y; sz += hit->endpoint.z;
        if (std::abs(x) > endpoint_error && std::abs(y) > endpoint_error)
            quadrants |= 1U << ((x >= 0 ? 1U : 0U) + (y >= 0 ? 2U : 0U));
    }
    if (quadrants != 15U) return patch;
    const double count = static_cast<double>(hits.size());
    const double mx = sx / count, my = sy / count, mz = sz / count;
    double xx = 0, xy = 0, yy = 0, xz = 0, yz = 0;
    for (const auto* hit : hits) {
        budget.Spend();
        const double x = hit->endpoint.x - cx - mx, y = hit->endpoint.y - cy - my;
        const double z = hit->endpoint.z - mz;
        xx += x*x; xy += x*y; yy += y*y; xz += x*z; yz += y*z;
    }
    xx /= count; xy /= count; yy /= count; xz /= count; yz /= count;
    const double eigen_min = (xx + yy - std::hypot(xx - yy, 2*xy)) / 2;
    const double determinant = xx*yy - xy*xy;
    if (!Positive(determinant) || eigen_min < options.minimum_covariance * resolution * resolution)
        return patch;
    patch.slope_x = (xz*yy - yz*xy) / determinant;
    patch.slope_y = (yz*xx - xz*xy) / determinant;
    patch.center_height = mz - patch.slope_x*mx - patch.slope_y*my;
    const double slope = std::hypot(patch.slope_x, patch.slope_y);
    if (!std::isfinite(patch.center_height) || !std::isfinite(slope) ||
        std::atan(slope) + options.calibration.gravity_angle_error > options.maximum_slope) return patch;
    double residual = 0;
    for (const auto* hit : hits) {
        budget.Spend();
        residual = std::max(residual, std::abs(hit->endpoint.z -
            Height(patch, hit->endpoint.x - cx, hit->endpoint.y - cy)));
    }
    if (!std::isfinite(residual) || residual > options.maximum_residual) return patch;
    patch.height_error = residual + endpoint_error * (1 + slope);
    patch.supported = std::isfinite(patch.height_error);
    return patch;
}

/** @brief Clamped XY closest-point parameter; stationary rotation remains a point capsule. */
double Parameter(double x, double y, const perception::Vec3& begin, const perception::Vec3& end) {
    const double dx = end.x - begin.x, dy = end.y - begin.y;
    const double squared = dx*dx + dy*dy;
    if (squared == 0) return 0;
    return std::clamp(((x-begin.x)*dx + (y-begin.y)*dy) / squared, 0.0, 1.0);
}
}  // namespace

GroundPatch LocalGroundSnapshot::Patch(unsigned x, unsigned y, double now) const {
    if (!std::isfinite(now) || now < built_at_ || now > valid_until_ ||
        x >= geometry_->options().dimensions[0] || y >= geometry_->options().dimensions[1]) return {};
    return patches_[Index(geometry_->options(), x, y)];
}

LocalGroundBuildResult BuildLocalGround(const std::shared_ptr<const LocalEsdfSnapshot>& geometry,
                                       const LocalGroundOptions& options, const ExecutionClock& clock) {
    LocalGroundBuildResult result;
    std::unique_ptr<GroundBudget> budget;
    try {
        const auto& calibration = options.calibration;
        if (!geometry || !clock || !calibration.healthy || !calibration.gravity_aligned ||
            calibration.execution_frame != geometry->options().execution_frame ||
            calibration.clock_domain != geometry->options().clock_domain ||
            calibration.alignment_epoch != geometry->alignment_epoch() ||
            !std::isfinite(calibration.valid_from) || !std::isfinite(calibration.valid_until) ||
            calibration.valid_from < 0 || calibration.valid_until < calibration.valid_from ||
            !std::isfinite(calibration.reference_height) || calibration.reference_height < 0 ||
            !std::isfinite(calibration.height_error) || calibration.height_error < 0 ||
            !std::isfinite(calibration.gravity_angle_error) || calibration.gravity_angle_error < 0 ||
            options.minimum_samples < 4 || options.minimum_samples > 500000 ||
            !Positive(options.minimum_quality) || options.minimum_quality > 1 ||
            !Positive(options.minimum_covariance) || options.minimum_covariance > .25 ||
            !Positive(options.maximum_residual) || !Positive(options.maximum_slope) ||
            options.maximum_slope >= 1.5 || calibration.gravity_angle_error > options.maximum_slope ||
            !Positive(options.maximum_reference_error) || !Positive(options.maximum_step) ||
            !Positive(options.maximum_drop) || !options.maximum_work || options.maximum_work > 100000000 ||
            !std::isfinite(options.deadline)) throw std::invalid_argument("invalid measured ground calibration or limits");
        const double initial = clock();
        if (!std::isfinite(initial) || initial < geometry->built_at() || initial > geometry->valid_until() ||
            initial < calibration.valid_from || initial > calibration.valid_until || initial >= options.deadline)
            throw std::invalid_argument("expired or regressed measured ground construction");
        budget = std::make_unique<GroundBudget>(options.maximum_work, options.deadline, clock, initial);
        auto snapshot = std::shared_ptr<LocalGroundSnapshot>(new LocalGroundSnapshot());
        snapshot->geometry_ = geometry; snapshot->options_ = options;
        snapshot->valid_until_ = std::min(geometry->valid_until(), calibration.valid_until);
        const auto& g = geometry->options();
        const auto cells = static_cast<std::size_t>(g.dimensions[0]) * g.dimensions[1];
        std::vector<std::vector<const MeasuredSurfaceSample*>> bins(cells);
        snapshot->patches_.resize(cells);
        for (std::size_t i = 0; i < cells; ++i) budget->Spend();
        for (const auto& hit : geometry->surface_samples()) {
            budget->Spend();
            if (hit.quality < options.minimum_quality) continue;
            const double gx = (hit.endpoint.x - g.origin.x) / g.resolution;
            const double gy = (hit.endpoint.y - g.origin.y) / g.resolution;
            const double gz = (hit.endpoint.z - g.origin.z) / g.resolution;
            if (gx < 0 || gy < 0 || gz < 0 || gx >= g.dimensions[0] || gy >= g.dimensions[1] || gz >= g.dimensions[2])
                continue;
            bins[Index(g, static_cast<unsigned>(gx), static_cast<unsigned>(gy))].push_back(&hit);
        }
        for (unsigned y = 0; y < g.dimensions[1]; ++y)
            for (unsigned x = 0; x < g.dimensions[0]; ++x) {
                budget->Spend();
                auto& patch = snapshot->patches_[Index(g, x, y)];
                patch = Fit(bins[Index(g, x, y)], g.origin.x + (x+.5)*g.resolution,
                    g.origin.y + (y+.5)*g.resolution, g.resolution,
                    g.measurement_error + geometry->alignment_error(), options, *budget);
                if (patch.supported) ++result.supported_patches;
            }
        snapshot->built_at_ = budget->Check();
        if (snapshot->built_at_ > snapshot->valid_until_)
            throw std::runtime_error("measured ground expired before publication");
        result.snapshot = std::move(snapshot);
    } catch (const std::exception& error) { result.reason = error.what(); }
    catch (...) { result.reason = "nonstandard measured ground construction exception"; }
    if (budget) result.work = budget->used();
    return result;
}

GroundSupportVerdict VerifyGroundSupport(const LocalGroundSnapshot& ground,
                                       const GroundSupportQuery& query, const ExecutionClock& clock) {
    const auto& geometry = *ground.geometry();
    const auto& g = geometry.options();
    const auto& options = ground.options();
    const auto& q = query.sweep;
    GroundSupportVerdict verdict;
    verdict.frame_id = g.execution_frame; verdict.alignment_epoch = geometry.alignment_epoch();
    verdict.geometry_revision = g.revision; verdict.valid_until = ground.valid_until();
    std::unique_ptr<GroundBudget> budget;
    try {
        const perception::Vec3 begin{q.begin.position[0], q.begin.position[1], q.begin.position[2]};
        const perception::Vec3 end{q.end.position[0], q.end.position[1], q.end.position[2]};
        const double radius = query.expanded_body.horizontal_radius;
        if (!clock || !begin.Finite() || !end.Finite() || !Positive(radius) || !query.maximum_work ||
            query.maximum_work > 100000000 || !std::isfinite(q.deadline) ||
            !std::isfinite(q.begin_stamp) || !std::isfinite(q.end_stamp) || q.end_stamp < q.begin_stamp ||
            !std::isfinite(q.linear_deviation) || q.linear_deviation < 0 ||
            !std::isfinite(query.stopping_distance) || query.stopping_distance < 0 ||
            !std::isfinite(query.reference_position_error) || query.reference_position_error < 0 ||
            query.stopping_distance > radius || q.geometry_revision != g.revision ||
            q.timing.frame_id != g.execution_frame || q.timing.alignment_epoch != geometry.alignment_epoch() ||
            q.begin_stamp < ground.built_at() || q.end_stamp > ground.valid_until()) return verdict;
        const double initial = clock();
        if (!std::isfinite(initial) || initial < ground.built_at() || initial > ground.valid_until() || initial >= q.deadline)
            return verdict;
        budget = std::make_unique<GroundBudget>(query.maximum_work, q.deadline, clock, initial);
        const double min_x = std::min(begin.x, end.x)-radius, max_x = std::max(begin.x, end.x)+radius;
        const double min_y = std::min(begin.y, end.y)-radius, max_y = std::max(begin.y, end.y)+radius;
        if (!std::isfinite(min_x) || !std::isfinite(max_x) || !std::isfinite(min_y) || !std::isfinite(max_y) ||
            min_x <= g.origin.x || min_y <= g.origin.y ||
            max_x >= g.origin.x + g.dimensions[0]*g.resolution ||
            max_y >= g.origin.y + g.dimensions[1]*g.resolution) return verdict;
        const auto first_x = static_cast<unsigned>(std::floor(std::nextafter((min_x-g.origin.x)/g.resolution,
            -std::numeric_limits<double>::infinity())));
        const auto first_y = static_cast<unsigned>(std::floor(std::nextafter((min_y-g.origin.y)/g.resolution,
            -std::numeric_limits<double>::infinity())));
        const auto last_x = static_cast<unsigned>(std::floor((max_x-g.origin.x)/g.resolution));
        const auto last_y = static_cast<unsigned>(std::floor((max_y-g.origin.y)/g.resolution));
        const double half = g.resolution / 2;
        const double diagonal = std::sqrt(2.0)*half;
        const double xy_length = std::hypot(end.x-begin.x, end.y-begin.y);
        const auto intersects = [&](unsigned x, unsigned y) {
            const double cx = g.origin.x + (x+.5)*g.resolution, cy = g.origin.y + (y+.5)*g.resolution;
            const double t = Parameter(cx, cy, begin, end);
            // The cell circumscribed disk includes every intersecting cell AABB.
            return std::hypot(cx-begin.x-(end.x-begin.x)*t, cy-begin.y-(end.y-begin.y)*t) <= radius+diagonal;
        };
        for (unsigned y = first_y; y <= last_y; ++y)
            for (unsigned x = first_x; x <= last_x; ++x) {
                budget->Spend();
                if (!intersects(x,y)) continue;
                const auto patch = ground.Patch(x,y,initial);
                if (!patch.supported) { verdict.work = budget->used(); return verdict; }
                const double cx = g.origin.x + (x+.5)*g.resolution, cy = g.origin.y + (y+.5)*g.resolution;
                const double t = Parameter(cx,cy,begin,end);
                const double reference = begin.z + (end.z-begin.z)*t - options.calibration.reference_height;
                // Bound patch/reference variation over the whole cell, including
                // clamped closest-point parameters and stationary vertical motion.
                const double reference_spread = xy_length > 0 ?
                    std::abs(end.z-begin.z)*std::min(1.0,diagonal/xy_length) : std::abs(end.z-begin.z);
                const double error = std::abs(patch.center_height-reference) +
                    half*(std::abs(patch.slope_x)+std::abs(patch.slope_y)) + reference_spread +
                    patch.height_error + options.calibration.height_error + q.linear_deviation +
                    query.reference_position_error +
                    (radius+diagonal)*std::tan(options.calibration.gravity_angle_error);
                if (!std::isfinite(error) || error > options.maximum_reference_error) {
                    verdict.status = GeometryStatus::kBlocked; verdict.work = budget->used(); return verdict;
                }
                for (unsigned axis = 0; axis < 2; ++axis) {
                    budget->Spend();
                    const unsigned nx = x+(axis==0), ny = y+(axis==1);
                    if (nx > last_x || ny > last_y || !intersects(nx,ny)) continue;
                    const auto neighbor = ground.Patch(nx,ny,initial);
                    if (!neighbor.supported) { verdict.work = budget->used(); return verdict; }
                    // Difference of the two fitted planes is linear along their
                    // common edge, so its extrema are the two edge endpoints.
                    for (double edge : {-half,half}) {
                        budget->Spend();
                        const double a = Height(patch,axis==0 ? half : edge,axis==0 ? edge : half);
                        const double b = Height(neighbor,axis==0 ? -half : edge,axis==0 ? edge : -half);
                        const double jump = std::abs(a-b) + patch.height_error + neighbor.height_error;
                        if (jump > std::min(options.maximum_step,options.maximum_drop)) {
                            verdict.status = GeometryStatus::kBlocked; verdict.work = budget->used(); return verdict;
                        }
                    }
                }
            }
        if (budget->Check() > ground.valid_until()) {
            verdict.work = budget->used(); return verdict;
        }
        verdict.status = GeometryStatus::kClear;
    } catch (...) { verdict.status = GeometryStatus::kIndeterminate; }
    if (budget) verdict.work = budget->used();
    return verdict;
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
