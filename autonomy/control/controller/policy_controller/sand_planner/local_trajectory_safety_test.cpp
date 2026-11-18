/* Copyright 2026 The Openbot Authors. */
/** @file local_trajectory_safety_test.cpp
 * @brief Real decoded depth/cloud inputs exercise immutable ESDF and body sweeps.
 * @details Acquisition/alignment attestations and measured-ground replies are
 * controlled independent dependencies. This suite does not certify real SLAM,
 * ground reconstruction, nonholonomic motion, dynamic objects or a robot tracker.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/local_trajectory_safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace p = autonomy::perception;
namespace s = autonomy::control::controller::policy_controller::sand_planner;
namespace {
/** @brief Assertions exercise independent behavior rather than debug-only assert. */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
/** @brief Independent little-endian FLOAT32 wire encoding. */
void Float(std::string& bytes, std::size_t offset, float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i) bytes[offset + i] = static_cast<char>((bits >> (8 * i)) & 255);
}
/** @brief Controlled healthy owner proof for each actual acquisition time. */
p::AcquisitionLocalization Pose(p::SensorKind sensor, const std::string& frame, double stamp) {
    p::AcquisitionLocalization proof;
    proof.world_frame = "map"; proof.sensor_frame = frame;
    proof.map_epoch = 7; proof.calibration_revision = 3;
    proof.support = p::Mask(sensor); proof.healthy = true;
    proof.valid_from = stamp - 1; proof.valid_until = stamp + 3;
    proof.world_from_sensor.translation.z = .5;
    if (sensor == p::SensorKind::kRgbd)
        proof.world_from_sensor.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    return proof;
}
/** @brief Explicit continuous alignment, distinct from the observation map epoch. */
s::ExecutionAlignment Alignment() {
    s::ExecutionAlignment a;
    a.world_frame = "map"; a.execution_frame = "odom"; a.clock_domain = "test_sensor_execution";
    a.map_epoch = 7; a.alignment_epoch = 9; a.valid_from = 9; a.valid_until = 13; a.healthy = true;
    return a;
}
/** @brief Real depth raster viewing a far vertical wall; masked pixels emit no rays. */
p::ObservationPacket Depth(bool hole = false, double stamp = 10, std::uint64_t sequence = 1) {
    constexpr unsigned width = 81;
    std::string bytes(width * width * 4, '\0');
    for (unsigned row = 0; row < width; ++row)
        for (unsigned column = 0; column < width; ++column)
            Float(bytes, (row * width + column) * 4,
                  hole && row >= 31 && row <= 49 ? std::numeric_limits<float>::quiet_NaN() : 4.0F);
    p::DepthImageBytes image{width, width, width * 4, false, "32FC1", "depth", stamp, bytes};
    p::DepthCalibration calibration{{81, 81, 40, 40, 40, 40, .1, 8}, "depth", 3, true};
    p::SensorDecodingOptions options; options.valid_for = 3;
    p::ObservationPacket packet;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly, options, Pose,
        [&](auto value, double) { packet = std::move(value); return true; }, [] { return 10.05; });
    Require(ingress.PushDepth(image, calibration, sequence) == p::ObservationIngressResult::kAccepted,
            "actual metric depth bytes admitted");
    return packet;
}
/** @brief Native cloud records retain actual time offsets; no free rays are invented. */
p::ObservationPacket Cloud(const std::vector<p::Vec3>& points, double stamp = 10,
                           bool rolling = false, std::uint64_t sequence = 1) {
    const unsigned stride = rolling ? 16 : 12;
    std::string bytes(points.size() * stride, '\0');
    for (std::size_t i = 0; i < points.size(); ++i) {
        Float(bytes, i * stride, points[i].x); Float(bytes, i * stride + 4, points[i].y);
        Float(bytes, i * stride + 8, points[i].z);
        if (rolling) Float(bytes, i * stride + 12, static_cast<float>(.001 * i));
    }
    p::PointCloudBytes cloud{static_cast<unsigned>(points.size()), 1, stride,
        static_cast<unsigned>(bytes.size()), false, "lidar", stamp,
        {{"x", 0, p::FieldScalar::kFloat32, 1}, {"y", 4, p::FieldScalar::kFloat32, 1},
         {"z", 8, p::FieldScalar::kFloat32, 1}}, bytes};
    if (rolling) cloud.fields.push_back({"time", 12, p::FieldScalar::kFloat32, 1});
    p::SensorDecodingOptions options; options.valid_for = 3; options.instantaneous_cloud = !rolling;
    p::ObservationPacket packet;
    p::ObservationIngress ingress(p::SensorMode::kLidarOnly, options, Pose,
        [&](auto value, double) { packet = std::move(value); return true; }, [] { return 10.05; });
    Require(ingress.PushCloud(cloud, sequence) == p::ObservationIngressResult::kAccepted,
            "actual cloud bytes and acquisition poses admitted");
    return packet;
}
/** @brief LiDAR far-wall measurements with the same physical angular coverage as depth. */
p::ObservationPacket LidarWall() {
    std::vector<p::Vec3> points;
    for (int row = 0; row <= 80; ++row)
        for (int column = 0; column <= 80; ++column)
            points.push_back({4, -(column - 40) * .1, -(row - 40) * .1});
    return Cloud(points);
}
/** @brief Fixed finite geometry volume and evidence horizon. */
s::LocalEsdfOptions Options(p::SensorMode mode = p::SensorMode::kRgbdOnly) {
    s::LocalEsdfOptions o;
    o.origin = {-1, -4, -2}; o.dimensions = {24, 32, 24}; o.resolution = .25;
    o.mode = mode; o.clock_domain = "test_sensor_execution"; o.revision = 17;
    o.maximum_observation_age = 3; o.measurement_error = .01; o.surface_margin = .02;
    o.deadline = 10.09;
    return o;
}
/** @brief Build failures are surfaced as missing snapshots, not partial geometry. */
std::shared_ptr<const s::LocalEsdfSnapshot> Build(const std::vector<p::ObservationPacket>& packets,
                                               const s::LocalEsdfOptions& o) {
    std::vector<s::ExecutionObservation> observations;
    for (const auto& packet : packets) observations.push_back({packet, Alignment()});
    auto result = s::BuildLocalEsdf(observations, o, [] { return 10.05; });
    if (!result.snapshot) throw std::runtime_error(result.reason);
    return result.snapshot;
}
/** @brief Analytical linear cubic Bezier, optionally with stationary rotation. */
s::TimedBSpline Reference(double begin = 1.5, double end = 1.7, double yaw = 0) {
    s::BSplineData data;
    for (unsigned i = 0; i < 4; ++i) data.control_points.push_back({begin + (end - begin) * i / 3.0, 0, 0, yaw * i / 3.0});
    data.knots = {0, 0, 0, 0, 1, 1, 1, 1};
    data.timing = {"odom", "test_lease", 9, 1, 10.1, 1, 13};
    return s::TimedBSpline(std::move(data));
}
/** @brief Explicit body and physical reaction/braking parameters for these fixtures. */
s::LocalTrajectorySafetyOptions Safety() {
    s::LocalTrajectorySafetyOptions o;
    o.body = {.12, .2, .8}; o.position_margin = .01;
    o.clock_domain = "test_sensor_execution";
    return o;
}
/** @brief Controlled ground evidence checks identity and finite stopping/body bounds.
 * This dependency deliberately does not claim production measured floor coverage.
 */
s::GroundSupportVerifier Ground() {
    return [](const s::GroundSupportQuery& q) {
        Require(q.expanded_body.horizontal_radius >= .12 && std::isfinite(q.stopping_distance),
                "ground producer receives complete expanded stopping footprint");
        return s::GroundSupportVerdict{s::GeometryStatus::kClear, q.sweep.timing.frame_id,
            q.sweep.timing.alignment_epoch, q.sweep.geometry_revision, 13};
    };
}
/** @brief Exercise the concrete backend and the real adaptive trajectory validator together. */
s::LocalTrajectorySafetyResult Check(const std::shared_ptr<const s::LocalEsdfSnapshot>& snapshot,
                                    const s::TimedBSpline& trajectory = Reference(),
                                    const s::LocalTrajectorySafetyOptions& body = Safety(),
                                    const s::GroundSupportVerifier& ground = Ground()) {
    s::TrajectoryValidationOptions validation;
    validation.geometry_revision = snapshot->options().revision; validation.deadline = 10.09;
    return s::ValidateLocalTrajectory(trajectory, {3, 10, 100, 4, 10, 100}, validation,
                                     body, snapshot, ground, [] { return 10.05; });
}

/** @brief Each supported input mode builds real geometry and admits a known-space body sweep. */
void ThreeModes() {
    const auto depth = Depth(), lidar = LidarWall();
    for (auto mode : {p::SensorMode::kRgbdOnly, p::SensorMode::kLidarOnly, p::SensorMode::kLidarRgbd}) {
        std::vector<p::ObservationPacket> packets;
        if (p::Mask(mode) & p::Mask(p::SensorKind::kRgbd)) packets.push_back(depth);
        if (p::Mask(mode) & p::Mask(p::SensorKind::kLidar)) packets.push_back(lidar);
        const auto snapshot = Build(packets, Options(mode));
        const auto result = Check(snapshot);
        Require(result.validation.admitted && result.support_queries && result.sweep_samples,
                "three modes admit the actual observed full body with controlled ground proof");
    }
    auto o = Options(p::SensorMode::kLidarRgbd);
    Require(!s::BuildLocalEsdf({{depth, Alignment()}}, o, [] { return 10.05; }).snapshot,
            "joint source policy does not silently downgrade missing LiDAR");
}

/** @brief Independent brute-force blockers verify the corrected EDT distance at many points. */
void DistanceOracle() {
    const auto snapshot = Build({Depth()}, Options());
    const auto& o = snapshot->options();
    std::vector<p::Vec3> blockers;
    for (unsigned z = 0; z < o.dimensions[2]; ++z)
        for (unsigned y = 0; y < o.dimensions[1]; ++y)
            for (unsigned x = 0; x < o.dimensions[0]; ++x)
                if (snapshot->State(x, y, z, 10.05) != s::LocalVoxelState::kFree)
                    blockers.push_back(snapshot->Center(x, y, z));
    for (unsigned z : {8U, 9U, 10U, 11U})
        for (unsigned y : {12U, 15U, 16U, 19U})
            for (unsigned x : {6U, 9U, 12U, 15U}) {
                auto center = snapshot->Center(x, y, z);
                if (snapshot->State(x, y, z, 10.05) != s::LocalVoxelState::kFree) continue;
                for (auto offset : {p::Vec3{}, p::Vec3{.04, -.02, .03}}) {
                    const auto point = center + offset;
                    double nearest = std::numeric_limits<double>::infinity();
                    for (const auto& blocked : blockers) nearest = std::min(nearest, (blocked - center).Norm());
                    const double boundary = std::min({point.x + 1, 5 - point.x, point.y + 4, 4 - point.y,
                                                      point.z + 2, 4 - point.z});
                    const double expected = std::max(0.0, std::min(nearest - std::sqrt(3.0) * .125 - offset.Norm(),
                                                                  boundary) - o.measurement_error);
                    Require(std::abs(snapshot->DistanceLowerBound(point, 10.05) - expected) < 1e-10,
                            "separable EDT matches independent brute-force corrected-distance oracle");
                }
            }
    Require(snapshot->DistanceLowerBound({100, 0, .5}, 10.05) == 0 &&
            snapshot->DistanceLowerBound({1.5, 0, .5}, 9.9) == 0,
            "exterior and regressed-time distances are never optimistic");
}

/** @brief Missing depth stripe cannot be filled by features, neighboring pixels or ray absence. */
void UnknownHoles() {
    const auto packet = Depth(true);
    Require(packet.observation.valid_fraction < 1 && packet.observation.rays.size() < 81 * 81,
            "masked wire samples emitted no invented rays");
    const auto snapshot = Build({packet}, Options());
    Require(!Check(snapshot).validation.admitted, "unknown body-space hole denies execution");
}

/** @brief A retained hit defeats clearing rays irrespective of source/input ordering. */
void OccupiedConflictAndOverhang() {
    const auto depth = Depth();
    const auto overhang = Cloud({{1.6, 0, .2}});
    auto o = Options(p::SensorMode::kLidarRgbd);
    const auto first = Build({depth, overhang}, o), reverse = Build({overhang, depth}, o);
    for (unsigned z = 0; z < o.dimensions[2]; ++z)
        for (unsigned y = 0; y < o.dimensions[1]; ++y)
            for (unsigned x = 0; x < o.dimensions[0]; ++x)
                Require(first->State(x, y, z, 10.05) == reverse->State(x, y, z, 10.05),
                        "occupied/free conflict is independent of input order");
    Require(!Check(first).validation.admitted && !Check(reverse).validation.admitted,
            "overhang inside robot height denies body sweep despite free low centerline");
    Require(first->DistanceLowerBound({1.6, 0, .1}, 10.05) > 0,
            "centerline below obstacle is free but cannot substitute for full body");
}

/** @brief A mid-route measured obstacle must be found even with safe endpoints. */
void BetweenEndpoints() {
    const auto snapshot = Build({Depth(), Cloud({{1.9, 0, 0}})}, Options(p::SensorMode::kLidarRgbd));
    auto body = Safety(); body.reaction_time = 0; body.minimum_braking_deceleration = 100;
    Require(Check(snapshot, Reference(1.25, 1.25), body).validation.admitted &&
            Check(snapshot, Reference(2.5, 2.5), body).validation.admitted,
            "separate stationary endpoints are actually body-safe");
    Require(!Check(snapshot, Reference(1.25, 2.5), body).validation.admitted,
            "continuous sweep catches obstacle between safe endpoints");
    const auto curved_obstacle = Build({Depth(), Cloud({{1.9, .675, 0}})}, Options(p::SensorMode::kLidarRgbd));
    Require(Check(curved_obstacle, Reference(1.5, 2.3), body).validation.admitted,
            "straight chord is actually body-safe beside curved-route obstacle");
    auto curve = Reference(1.5, 2.3).data();
    curve.control_points[1][1] = curve.control_points[2][1] = .9;
    Require(!Check(curved_obstacle, s::TimedBSpline(std::move(curve)), body).validation.admitted,
            "actual spline bulge cannot be replaced by the safe endpoint chord");
}

/** @brief Stationary yaw motion still verifies the entire robot's circumscribed body. */
void RotationAndBodyRadius() {
    const auto clear = Build({Depth()}, Options());
    Require(Check(clear, Reference(1.5, 1.5, 3)).validation.admitted, "known safe stationary rotation admitted");
    const auto obstacle = Build({Depth(), Cloud({{1.5, .25, 0}})}, Options(p::SensorMode::kLidarRgbd));
    auto body = Safety(); body.body.horizontal_radius = .3;
    Require(!Check(obstacle, Reference(1.5, 1.5, 3), body).validation.admitted,
            "rotation never bypasses protrusion/body-radius checks");
}

/** @brief Unknown exterior remains blocked after clipping incoming rays to a smaller volume. */
void BoundaryClipping() {
    auto o = Options(); o.origin.x = 1; o.dimensions[0] = 8;
    const auto snapshot = Build({Depth()}, o);
    Require(Check(snapshot).validation.admitted, "sensor origin outside local volume clips valid ray interiors");
    Require(!Check(snapshot, Reference(2.95, 2.95)).validation.admitted &&
            snapshot->DistanceLowerBound({3.1, 0, .5}, 10.05) == 0,
            "body touching local-volume exterior denies execution");
}

/** @brief Earliest real acquisition expiry controls snapshots and cannot be renewed by rebuilding. */
void ExpiryAndRollingProofs() {
    const auto packet = Cloud({{2, 0, 0}, {2, .2, 0}}, 10, true);
    Require(packet.acquisition_poses.size() == 2 && packet.observation.rays[0].stamp < packet.observation.stamp,
            "rolling input retains distinct actual beam proofs");
    auto o = Options(p::SensorMode::kLidarOnly); o.maximum_observation_age = .1;
    const auto snapshot = Build({packet}, o);
    Require(std::abs(snapshot->valid_until() - 10.1) < 1e-9,
            "earliest beam, not scan-end timestamp, limits local geometry age");
    Require(!Check(snapshot).validation.admitted, "trajectory beyond evidence expiry is rejected");
    Require(!s::BuildLocalEsdf({{packet, Alignment()}}, o, [] { return 10.11; }).snapshot,
            "rebuilding a scan does not renew early beam evidence");
    auto truncated = packet; truncated.acquisition_poses.erase(truncated.acquisition_poses.begin());
    Require(!s::BuildLocalEsdf({{truncated, Alignment()}}, o, [] { return 10.05; }).snapshot,
            "local ESDF requires every individual acquisition proof");
    auto alignment = Alignment(); alignment.valid_from = 10.0005;
    Require(!s::BuildLocalEsdf({{packet, alignment}}, o, [] { return 10.05; }).snapshot,
            "scan-end alignment cannot attest an earlier beam outside support");
}

/** @brief Frame, epoch, fixed transform and explicit clock domain must all match. */
void AlignmentIdentity() {
    auto packet = Depth(); auto a = Alignment(); auto o = Options();
    a.healthy = false;
    Require(!s::BuildLocalEsdf({{packet, a}}, o, [] { return 10.05; }).snapshot, "unhealthy alignment rejected");
    a = Alignment(); a.clock_domain = "other";
    Require(!s::BuildLocalEsdf({{packet, a}}, o, [] { return 10.05; }).snapshot, "unattested cross-clock mixing rejected");
    auto later = Depth(false, 10.01, 2); auto changed = Alignment(); changed.execution_from_world.translation.x = 1;
    Require(!s::BuildLocalEsdf({{packet, Alignment()}, {later, changed}}, o, [] { return 10.05; }).snapshot,
            "fixed transform cannot change within one execution epoch");
    changed = Alignment(); changed.alignment_epoch = 10;
    Require(!s::BuildLocalEsdf({{packet, Alignment()}, {later, changed}}, o, [] { return 10.05; }).snapshot,
            "mixed continuous alignment epochs rejected");
    auto rotated = Alignment(); rotated.execution_from_world.rotation = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    rotated.execution_from_world.translation = {1, 0, 0};
    auto build = s::BuildLocalEsdf({{packet, rotated}}, o, [] { return 10.05; });
    Require(build.snapshot && build.snapshot->DistanceLowerBound({1, 1.5, .5}, 10.05) > 0 &&
            build.snapshot->DistanceLowerBound({-1, 1.5, .5}, 10.05) == 0,
            "actual rigid conversion is applied instead of equating map with odom");
    auto reference = Reference().data(); reference.timing.frame_id = "map";
    Require(!Check(Build({packet}, o), s::TimedBSpline(reference)).validation.admitted,
            "reference frame cannot bypass explicit execution-frame identity");
}

/** @brief Conflicting acquisitions cannot share a single immutable evidence identity. */
void SourceIdentityConsistency() {
    const auto depth = Depth(); auto o = Options();
    Require(!s::BuildLocalEsdf({{depth, Alignment()}, {depth, Alignment()}}, o,
                              [] { return 10.05; }).snapshot,
            "duplicate source ordinal must be rejected rather than counted twice");
    auto changed_calibration = Depth(false, 10.01, 2);
    changed_calibration.localization.calibration_revision = 4;
    Require(changed_calibration.Valid(10.05), "individual new-calibration packet is independently valid");
    Require(!s::BuildLocalEsdf({{depth, Alignment()}, {changed_calibration, Alignment()}}, o,
                              [] { return 10.05; }).snapshot,
            "one active source cannot mix calibration generations inside one snapshot");
    auto lidar = Cloud({{4, 0, 0}});
    lidar.localization.world_from_base.translation.x = 1;
    for (auto& sample : lidar.acquisition_poses) sample.localization.world_from_base.translation.x = 1;
    Require(lidar.Valid(10.05), "independent packet remains internally self-consistent");
    o.mode = p::SensorMode::kLidarRgbd;
    Require(!s::BuildLocalEsdf({{depth, Alignment()}, {lidar, Alignment()}}, o,
                              [] { return 10.05; }).snapshot,
            "same acquisition time cannot attest conflicting base poses across sources");
}

/** @brief A ray through a voxel corner cannot create the adjacent unobserved volume. */
void CornerTraversal() {
    auto packet = Cloud({{3, 3, 3}});
    // Translate the already proven measurement and every matching pose together.
    const p::Vec3 shift{.5, .5, 0};
    packet.localization.world_from_sensor.translation = packet.localization.world_from_sensor.translation + shift;
    for (auto& proof : packet.acquisition_poses)
        proof.localization.world_from_sensor.translation = proof.localization.world_from_sensor.translation + shift;
    for (auto& ray : packet.observation.rays) ray.origin = ray.origin + shift;
    Require(packet.Valid(10.05), "corner fixture retains all exact sensor origin attestations");
    auto o = Options(p::SensorMode::kLidarOnly); o.origin = {0, 0, 0}; o.dimensions = {4, 4, 4};
    o.resolution = 1; o.surface_margin = 0; o.measurement_error = 0;
    const auto snapshot = Build({packet}, o);
    Require(snapshot->State(0, 0, 0, 10.05) == s::LocalVoxelState::kFree &&
            snapshot->State(1, 1, 1, 10.05) == s::LocalVoxelState::kFree &&
            snapshot->State(1, 0, 0, 10.05) == s::LocalVoxelState::kUnknown &&
            snapshot->State(0, 1, 0, 10.05) == s::LocalVoxelState::kUnknown &&
            snapshot->State(3, 3, 3, 10.05) == s::LocalVoxelState::kOccupied,
            "corner contact cannot carve neighboring cells or erase the measured hit");
}

/** @brief Missing/unknown/old/mismatched ground evidence never becomes a nominal floor. */
void GroundEvidence() {
    const auto snapshot = Build({Depth()}, Options());
    Require(!Check(snapshot, Reference(), Safety(), {}).validation.admitted, "missing ground producer denies admission");
    for (unsigned variant = 0; variant < 5; ++variant) {
        const auto ground = [variant](const s::GroundSupportQuery& q) {
            auto v = Ground()(q);
            if (variant == 0) v.status = s::GeometryStatus::kIndeterminate;
            if (variant == 1) v.valid_until = 10.2;
            if (variant == 2) v.frame_id = "map";
            if (variant == 3) ++v.alignment_epoch;
            if (variant == 4) ++v.geometry_revision;
            return v;
        };
        Require(!Check(snapshot, Reference(), Safety(), ground).validation.admitted,
                "ground support status/time/frame/epoch/revision all constrain admission");
    }
    const auto throwing = [](const s::GroundSupportQuery&) -> s::GroundSupportVerdict { throw 42; };
    Require(!Check(snapshot, Reference(), Safety(), throwing).validation.admitted, "ground exceptions deny reference");
}

/** @brief Braking expansion changes admissibility while preserving underlying geometry. */
void StoppingMargin() {
    const auto snapshot = Build({Depth()}, Options());
    auto body = Safety(); body.minimum_braking_deceleration = 100; body.reaction_time = 0;
    Require(Check(snapshot, Reference(2.5, 3), body).validation.admitted, "small guaranteed stopping footprint clear");
    body.minimum_braking_deceleration = .05;
    Require(!Check(snapshot, Reference(2.5, 3), body).validation.admitted,
            "insufficient stopping room rejects the same observed reference");
    body.minimum_braking_deceleration = 0;
    Require(!Check(snapshot, Reference(), body).validation.admitted, "no guaranteed braking capability rejected");
}

/** @brief Work, deadlines, malformed evidence and throwing/regressed clocks publish no safe partial. */
void BoundsAndClockFailures() {
    const auto packet = Depth(); auto o = Options(); o.maximum_work = 10;
    auto build = s::BuildLocalEsdf({{packet, Alignment()}}, o, [] { return 10.05; });
    Require(!build.snapshot && build.work == 10, "initialization participates in hard build budget");
    o = Options(); o.maximum_input_elements = 100;
    Require(!s::BuildLocalEsdf({{packet, Alignment()}}, o, [] { return 10.05; }).snapshot, "input bounded before packet validation");
    o = Options(); o.dimensions = {128, 128, 128};
    Require(!s::BuildLocalEsdf({{packet, Alignment()}}, o, [] { return 10.05; }).snapshot, "memory volume bound enforced");
    o = Options(); unsigned clock_calls = 0;
    Require(!s::BuildLocalEsdf({{packet, Alignment()}}, o, [&] { return ++clock_calls == 1 ? 10.05 : 10.04; }).snapshot,
            "build clock regression denies publication");
    clock_calls = 0;
    Require(!s::BuildLocalEsdf({{packet, Alignment()}}, o, [&] { return ++clock_calls == 1 ? 10.05 : 10.09; }).snapshot,
            "build deadline expiration denies publication");
    Require(!s::BuildLocalEsdf({{packet, Alignment()}}, o, []() -> double { throw 7; }).snapshot,
            "build clock exceptions are fail closed");
    auto forged = packet; forged.observation.rays[0].origin.x = 1;
    Require(!s::BuildLocalEsdf({{forged, Alignment()}}, o, [] { return 10.05; }).snapshot, "forged ray origins denied");
    const auto snapshot = Build({packet}, Options()); auto body = Safety(); body.maximum_work = 1;
    Require(!Check(snapshot, Reference(), body).validation.admitted, "entire body sweep has one aggregate work bound");
    // Each stationary chord consumes one sample and one support query. Two
    // chords use all four units; the third must fail the shared trajectory budget.
    body = Safety(); body.maximum_work = 4;
    const auto aggregated = Check(snapshot, Reference(1.5, 1.5), body);
    Require(!aggregated.validation.admitted && aggregated.support_queries >= 2,
            "work budget persists across individually affordable stationary chords");
}
}  // namespace

/** @brief Named cases report actual assertion outcomes and no production/robot claims. */
int main() {
    const std::vector<std::pair<const char*, void (*)()>> cases{
        {"three real input modes", ThreeModes}, {"independent distance oracle", DistanceOracle},
        {"unknown depth holes", UnknownHoles}, {"occupied conflict and overhang", OccupiedConflictAndOverhang},
        {"obstacle between safe endpoints", BetweenEndpoints}, {"rotation full body", RotationAndBodyRadius},
        {"ray and body boundary clipping", BoundaryClipping}, {"rolling beam provenance and expiry", ExpiryAndRollingProofs},
        {"execution alignment identity", AlignmentIdentity}, {"ground evidence fail closed", GroundEvidence},
        {"guaranteed stopping margin", StoppingMargin}, {"input work and clock bounds", BoundsAndClockFailures},
        {"source identity consistency", SourceIdentityConsistency}, {"corner traversal", CornerTraversal}};
    for (const auto& test : cases) {
        try { test.second(); std::cout << "PASS " << test.first << '\n'; }
        catch (const std::exception& exception) {
            std::cerr << "FAIL " << test.first << ": " << exception.what() << '\n'; return 1;
        }
    }
    return 0;
}
