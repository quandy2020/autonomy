/* Copyright 2026 The Openbot Authors. */
/** @file local_ground_test.cpp
 * @brief Real wire measurements exercise ground reconstruction and trajectory consumption.
 * @details Localization and gravity/calibration owners are controlled dependencies;
 * these tests do not certify real SLAM, material strength or robot tracking.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/local_ground.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace p = autonomy::perception;
namespace s = autonomy::control::controller::policy_controller::sand_planner;
namespace {
/** @brief Behavior assertions remain active in release builds. */
void Require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
/** @brief Independent little-endian FLOAT32 encoding of actual sensor wire records. */
void Float(std::string& bytes, std::size_t offset, float value) {
    std::uint32_t bits; std::memcpy(&bits,&value,sizeof(bits));
    for (unsigned i=0;i<4;++i) bytes[offset+i]=static_cast<char>((bits>>(8*i))&255);
}
/** @brief Controlled acquisition owner with a correctly downward optical camera transform. */
p::AcquisitionLocalization Pose(p::SensorKind sensor, const std::string& frame, double stamp) {
    p::AcquisitionLocalization pose;
    pose.world_frame="map"; pose.sensor_frame=frame; pose.map_epoch=7; pose.calibration_revision=3;
    pose.support=p::Mask(sensor); pose.healthy=true; pose.valid_from=stamp-1; pose.valid_until=stamp+3;
    pose.world_from_sensor.translation.z=2;
    if (sensor==p::SensorKind::kRgbd) pose.world_from_sensor.rotation={1,0,0,0,-1,0,0,0,-1};
    return pose;
}
/** @brief Explicit fixed alignment; no implicit gravity/IMU is inferred from this transform. */
s::ExecutionAlignment Alignment() {
    s::ExecutionAlignment alignment;
    alignment.world_frame="map"; alignment.execution_frame="odom"; alignment.clock_domain="ground_test_clock";
    alignment.map_epoch=7; alignment.alignment_epoch=9; alignment.healthy=true;
    alignment.valid_from=9; alignment.valid_until=13;
    return alignment;
}
/** @brief Wire raster of a real analytical surface viewed by the downward camera. */
p::ObservationPacket Depth(double height=0, double slope=0, bool hole=false) {
    constexpr unsigned width=81;
    std::string bytes(width*width*4,'\0');
    for (unsigned row=0;row<width;++row)
        for (unsigned column=0;column<width;++column) {
            const double optical_x=(static_cast<double>(column)-40)/40;
            const float depth=static_cast<float>((2-height)/(1+slope*optical_x));
            Float(bytes,(row*width+column)*4,hole && column>=35 && column<=45 ?
                std::numeric_limits<float>::quiet_NaN() : depth);
        }
    p::DepthImageBytes image{width,width,width*4,false,"32FC1","depth",10,bytes};
    p::DepthCalibration calibration{{81,81,40,40,40,40,.1,8},"depth",3,true};
    p::SensorDecodingOptions decoding; decoding.valid_for=3;
    p::ObservationPacket packet;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly,decoding,Pose,
        [&](auto value,double){packet=std::move(value);return true;},[]{return 10.05;});
    Require(ingress.PushDepth(image,calibration,1)==p::ObservationIngressResult::kAccepted,"real floor depth admitted");
    return packet;
}
/** @brief PointCloud2-style actual hit records; there are no synthesized no-return rays. */
p::ObservationPacket Cloud(const std::vector<p::Vec3>& points, std::uint64_t sequence=1) {
    std::string bytes(points.size()*12,'\0');
    for (std::size_t i=0;i<points.size();++i) {
        Float(bytes,i*12,points[i].x); Float(bytes,i*12+4,points[i].y); Float(bytes,i*12+8,points[i].z-2);
    }
    p::PointCloudBytes cloud{static_cast<unsigned>(points.size()),1,12,static_cast<unsigned>(bytes.size()),
        false,"lidar",10,{{"x",0,p::FieldScalar::kFloat32,1},{"y",4,p::FieldScalar::kFloat32,1},
        {"z",8,p::FieldScalar::kFloat32,1}},bytes};
    p::SensorDecodingOptions decoding; decoding.valid_for=3; decoding.instantaneous_cloud=true;
    p::ObservationPacket packet;
    p::ObservationIngress ingress(p::SensorMode::kLidarOnly,decoding,Pose,
        [&](auto value,double){packet=std::move(value);return true;},[]{return 10.05;});
    Require(ingress.PushCloud(cloud,sequence)==p::ObservationIngressResult::kAccepted,"real floor cloud admitted");
    return packet;
}
/** @brief Dense actual sample lattice; optional missing column is never substituted. */
std::vector<p::Vec3> Surface(double height=0,double slope=0,double step=0,bool hole=false) {
    std::vector<p::Vec3> points;
    for (int y=-39;y<=39;++y)
        for (int x=-39;x<=39;++x) {
            const double px=x*.05,py=y*.05;
            if (hole && px>=0 && px<.25) continue;
            points.push_back({px,py,height+slope*px+(px>=0 ? step : 0)});
        }
    return points;
}
/** @brief Explicit volume/time/source limits used by these measurement fixtures. */
s::LocalEsdfOptions GeometryOptions(p::SensorMode mode=p::SensorMode::kLidarOnly) {
    s::LocalEsdfOptions options;
    // Place the measured floor in a voxel interior. The original boundary-aligned
    // grid legitimately marked both adjoining cells occupied and denied the
    // fixture's low body; the safety rule is retained rather than waived.
    options.origin={-2,-2,-1.125}; options.dimensions={16,16,16}; options.resolution=.25;
    options.mode=mode; options.clock_domain="ground_test_clock"; options.revision=17;
    options.maximum_observation_age=3; options.measurement_error=.002; options.surface_margin=.005;
    options.deadline=10.09;
    return options;
}
/** @brief Construct geometry solely through the production validated observation consumer. */
std::shared_ptr<const s::LocalEsdfSnapshot> Geometry(const std::vector<p::ObservationPacket>& packets,
                                                  p::SensorMode mode=p::SensorMode::kLidarOnly) {
    std::vector<s::ExecutionObservation> inputs;
    for (const auto& packet:packets) inputs.push_back({packet,Alignment()});
    auto result=s::BuildLocalEsdf(inputs,GeometryOptions(mode),[]{return 10.05;});
    if (!result.snapshot) throw std::runtime_error(result.reason);
    return result.snapshot;
}
/** @brief Controlled calibrated reference height and explicitly attested gravity frame. */
s::LocalGroundOptions GroundOptions() {
    s::LocalGroundOptions options;
    options.calibration={"odom","ground_test_clock",9,9,13,0,.002,0,true,true};
    options.deadline=10.09;
    return options;
}
/** @brief Build real ground patches; no controlled floor verdict is injected. */
std::shared_ptr<const s::LocalGroundSnapshot> Ground(const std::shared_ptr<const s::LocalEsdfSnapshot>& geometry,
                                                   const s::LocalGroundOptions& options=GroundOptions()) {
    auto result=s::BuildLocalGround(geometry,options,[]{return 10.05;});
    if (!result.snapshot) throw std::runtime_error(result.reason);
    return result.snapshot;
}
/** @brief Entire measured-footprint query, including caller-applied stopping expansion. */
s::GroundSupportQuery Query(p::Vec3 begin={0,0,0},p::Vec3 end={.1,0,0},double radius=.12) {
    s::GroundSupportQuery query;
    query.sweep.begin.position={begin.x,begin.y,begin.z,0}; query.sweep.end.position={end.x,end.y,end.z,0};
    query.sweep.begin_stamp=10.1; query.sweep.end_stamp=11.1; query.sweep.geometry_revision=17;
    query.sweep.timing={"odom","test_reference",9,1,10.1,1,13}; query.sweep.deadline=10.09;
    query.expanded_body={radius,.2,.8}; query.maximum_work=2000000;
    return query;
}
/** @brief Real production ground verdict with a controlled authoritative clock. */
s::GroundSupportVerdict Check(const std::shared_ptr<const s::LocalGroundSnapshot>& ground,
                            const s::GroundSupportQuery& query=Query()) {
    return s::VerifyGroundSupport(*ground,query,[]{return 10.05;});
}
/** @brief Analytical cubic reference; physical duration is explicit. */
s::TimedBSpline Reference(double duration=1,bool stationary=false) {
    s::BSplineData data;
    for (unsigned i=0;i<4;++i) data.control_points.push_back({stationary ? 0 : .1*i/3,0,0,0});
    data.knots={0,0,0,0,1,1,1,1}; data.timing={"odom","test_reference",9,1,10.1,duration,13};
    return s::TimedBSpline(std::move(data));
}
/** @brief Explicit complete robot and physical braking calibration for admission fixtures. */
s::LocalTrajectorySafetyOptions Safety() {
    s::LocalTrajectorySafetyOptions options;
    options.body={.12,.2,.8}; options.position_margin=.005; options.clock_domain="ground_test_clock";
    return options;
}
/** @brief Production trajectory/ESDF/ground consumer, without any fake support callback. */
s::LocalTrajectorySafetyResult Admit(const std::shared_ptr<const s::LocalGroundSnapshot>& ground,
                                    const s::TimedBSpline& reference=Reference(),
                                    const s::LocalTrajectorySafetyOptions& safety=Safety()) {
    s::TrajectoryValidationOptions options; options.geometry_revision=17; options.deadline=10.09;
    return s::ValidateLocalTrajectory(reference,{3,10,100,4,10,100},options,safety,ground->geometry(),ground,[]{return 10.05;});
}

/** @brief Explicit calibrated planar differential-drive fixture, not implicit holonomic execution. */
s::DifferentialDriveLimits Drive() {
    s::DifferentialDriveLimits limits;
    limits.calibration_revision=7; limits.track_width=.4;
    limits.wheel_velocity={1,1}; limits.wheel_acceleration={3,3}; limits.wheel_jerk={10,10};
    limits.maximum_lateral_velocity=.005; limits.maximum_lateral_acceleration=.025;
    limits.maximum_vertical_velocity=1e-8;
    return limits;
}

/** @brief Production combined model/ESDF/ground path; all stages consume the same reference. */
s::LocalTrajectorySafetyResult AdmitDrive(const std::shared_ptr<const s::LocalGroundSnapshot>& ground,
                                         const s::TimedBSpline& reference=Reference(),
                                         const s::LocalTrajectorySafetyOptions& safety=Safety()) {
    s::TrajectoryValidationOptions options; options.geometry_revision=17; options.deadline=10.09;
    return s::ValidateLocalTrajectory(reference,{3,10,100,4,10,100},options,safety,
                                      ground->geometry(),ground,Drive(),[]{return 10.05;});
}

/** @brief Every supported source mode admits actual flat measured ground and body geometry. */
void FlatThreeModes() {
    const auto depth=Depth(),lidar=Cloud(Surface());
    for (auto mode:{p::SensorMode::kRgbdOnly,p::SensorMode::kLidarOnly,p::SensorMode::kLidarRgbd}) {
        std::vector<p::ObservationPacket> packets;
        if (p::Mask(mode)&p::Mask(p::SensorKind::kRgbd)) packets.push_back(depth);
        if (p::Mask(mode)&p::Mask(p::SensorKind::kLidar)) packets.push_back(lidar);
        const auto ground=Ground(Geometry(packets,mode));
        Require(Check(ground).status==s::GeometryStatus::kClear,"actual measured floor supports whole footprint");
        const auto result=AdmitDrive(ground);
        if (!result.validation.admitted)
            std::cerr<<"mode="<<static_cast<unsigned>(mode)<<" reason="<<result.validation.reason
                     <<" cells="<<result.examined_voxels<<" samples="<<result.sweep_samples
                     <<" ground="<<result.ground_work<<'\n';
        Require(result.validation.admitted && result.differential_drive.admitted &&
                result.ground_work && result.support_queries,
                "production drive, measured ground and complete body admit all three real wire-input modes");
    }
}

/** @brief Safe geometry does not authorize sideways motion or uncalibrated wheels. */
void DifferentialDriveConsumer() {
    const auto ground=Ground(Geometry({Cloud(Surface())}));
    auto data=Reference().data();
    for (auto& control:data.control_points) control[3]=1.5707963267948966;
    const s::TimedBSpline sideways(std::move(data));
    Require(Admit(ground,sideways).validation.admitted,"sideways fixture is actually geometry-safe");
    const auto denied=AdmitDrive(ground,sideways);
    Require(!denied.validation.admitted && !denied.differential_drive.admitted &&
            denied.validation.reason.find("lateral velocity")!=std::string::npos && !denied.ground_work,
            "actual combined consumer rejects nonholonomic motion before expensive geometry");
    data=Reference(1,true).data();
    for (unsigned i=0;i<4;++i) data.control_points[i][3]=.5*i/3;
    Require(AdmitDrive(ground,s::TimedBSpline(std::move(data))).validation.admitted,
            "measured body/ground and calibrated wheels support bounded stationary rotation");
    s::TrajectoryValidationOptions options; options.geometry_revision=17; options.deadline=10.09;
    auto drive=Drive(); drive.calibration_revision=0;
    Require(!s::ValidateLocalTrajectory(Reference(),{3,10,100,4,10,100},options,Safety(),
        ground->geometry(),ground,drive,[]{return 10.05;}).validation.admitted,
        "production drive overload requires actual calibration identity");
}

/** @brief Model work and cross-stage clock regressions cannot be hidden by restarting body checks. */
void SharedDriveBodyGroundBudget() {
    const auto ground=Ground(Geometry({Cloud(Surface())})); const auto reference=Reference();
    const auto geometric=Admit(ground,reference);
    Require(geometric.validation.admitted,"shared-budget baseline has genuinely safe geometry");
    auto safety=Safety();
    safety.maximum_work=geometric.examined_voxels+geometric.sweep_samples+
                        geometric.support_queries+geometric.ground_work;
    Require(Admit(ground,reference,safety).validation.admitted,"exact geometry-only budget is sufficient");
    Require(!AdmitDrive(ground,reference,safety).validation.admitted,
            "drive interval work reduces the same whole-trajectory body/ground allowance");
    s::TrajectoryValidationOptions options; options.geometry_revision=17; options.deadline=10.09;
    unsigned model_clock_calls=0;
    Require(s::ValidateDifferentialDrive(reference,Drive(),options,[&]{++model_clock_calls; return 10.05;}).admitted,
            "clock transition fixture completes its independent model proof");
    unsigned calls=0;
    const auto regression=s::ValidateLocalTrajectory(reference,{3,10,100,4,10,100},options,Safety(),
        ground->geometry(),ground,Drive(),[&]{return ++calls>model_clock_calls ? 10.04 : 10.05;});
    Require(regression.differential_drive.admitted && !regression.validation.admitted &&
            regression.validation.reason.find("clock")!=std::string::npos,
            "clock regression precisely between successful model and body stages denies");
    const auto combined=AdmitDrive(ground,reference);
    options.maximum_segments=combined.differential_drive.examined_intervals;
    Require(!s::ValidateLocalTrajectory(reference,{3,10,100,4,10,100},options,Safety(),
        ground->geometry(),ground,Drive(),[]{return 10.05;}).validation.admitted,
        "kinematic intervals cannot reset the aggregate model/geometric segment budget");
}
/** @brief Measured hit provenance and memory bounds cannot be fabricated or silently truncated. */
void ProvenanceAndMemory() {
    const auto packet=Depth(); const auto geometry=Geometry({packet},p::SensorMode::kRgbdOnly);
    Require(geometry->surface_samples().size()==packet.observation.rays.size(),"all real hits retained");
    const auto& sample=geometry->surface_samples()[123]; const auto& ray=packet.observation.rays[123];
    Require((sample.endpoint-ray.Endpoint()).Norm()<1e-12 && sample.sensor==p::SensorKind::kRgbd &&
        sample.sequence==1 && sample.calibration_revision==3 && sample.stamp==ray.stamp,
        "retained ground input preserves exact validated hit provenance");
    auto options=GeometryOptions(p::SensorMode::kRgbdOnly); options.maximum_surface_samples=10;
    const auto result=s::BuildLocalEsdf({{packet,Alignment()}},options,[]{return 10.05;});
    Require(!result.snapshot && result.reason.find("surface sample")!=std::string::npos,
        "retained hit overflow refuses the entire geometry publication");
}
/** @brief Camera and LiDAR fit a real gentle ramp; steep surfaces cannot become floor. */
void Ramps() {
    for (auto mode:{p::SensorMode::kRgbdOnly,p::SensorMode::kLidarOnly}) {
        const auto packet=mode==p::SensorMode::kRgbdOnly ? Depth(0,.12) : Cloud(Surface(0,.12));
        const auto ground=Ground(Geometry({packet},mode));
        Require(Check(ground,Query({0,0,0},{.1,0,.012})).status==s::GeometryStatus::kClear,"measured gentle ramp supports matching reference");
    }
    const auto steep=Ground(Geometry({Cloud(Surface(0,.6))}));
    Require(Check(steep).status!=s::GeometryStatus::kClear,"excess slope denies ground support");
}
/** @brief Missing depth pixels and missing point-cloud columns stay unsupported. */
void Holes() {
    Require(Check(Ground(Geometry({Depth(0,0,true)},p::SensorMode::kRgbdOnly))).status!=s::GeometryStatus::kClear,
            "invalid depth raster cannot synthesize floor");
    const auto ground=Ground(Geometry({Cloud(Surface(0,0,0,true))}));
    Require(Check(ground,Query({-.5,0,0},{-.5,0,0},.03)).status==s::GeometryStatus::kClear &&
        Check(ground,Query({.6,0,0},{.6,0,0},.03)).status==s::GeometryStatus::kClear,
        "separate endpoints have measured support");
    Require(Check(ground,Query({-.5,0,0},{.6,0,0},.03)).status!=s::GeometryStatus::kClear,
        "unknown floor between safe endpoints denies whole chord");
}
/** @brief A handful of points on one line cannot prove cell-area support. */
void SparseAndCollinear() {
    std::vector<p::Vec3> points;
    for (int i=-39;i<=39;++i) points.push_back({i*.05,.025,0});
    const auto ground=Ground(Geometry({Cloud(points)}));
    Require(Check(ground).status!=s::GeometryStatus::kClear,"collinear samples reject area support");
    const auto sparse=Ground(Geometry({Cloud({{-.05,-.05,0},{.05,.05,0}})}));
    Require(Check(sparse).status!=s::GeometryStatus::kClear,"two points do not synthesize a floor plane");
}
/** @brief A planar tabletop is measured but is not the calibrated supporting floor. */
void TabletopAndConflictingHeights() {
    const auto table=Ground(Geometry({Cloud(Surface(.6))}));
    Require(table->Patch(8,8,10.05).supported,"table is a genuinely measured planar surface");
    Require(Check(table).status==s::GeometryStatus::kBlocked,"calibrated height prevents tabletop being treated as floor");
    const auto conflict=Ground(Geometry({Cloud(Surface()),Cloud(Surface(.6),2)}));
    Require(Check(conflict).status!=s::GeometryStatus::kClear,"conflicting measured height layers cannot select a convenient floor");
}
/** @brief Vertical observed points do not turn into a traversable support patch. */
void Wall() {
    std::vector<p::Vec3> points;
    for (int y=-20;y<=20;++y) for (int z=-10;z<=20;++z) points.push_back({.125,y*.05,z*.05});
    Require(Check(Ground(Geometry({Cloud(points)}))).status!=s::GeometryStatus::kClear,"wall has insufficient stable XY support");
}
/** @brief Positive and negative floor discontinuities fail even when height tolerance is relaxed. */
void StepsAndDrops() {
    auto options=GroundOptions(); options.maximum_reference_error=.5;
    for (double step:{.12,-.12}) {
        const auto ground=Ground(Geometry({Cloud(Surface(0,0,step))}),options);
        Require(ground->Patch(7,8,10.05).supported && ground->Patch(8,8,10.05).supported,
                "two horizontal measured patches exist on either side of step/drop");
        Require(Check(ground,Query({0,0,0},{0,0,0})).status==s::GeometryStatus::kBlocked,
                "neighbor continuity independently rejects step and drop");
    }
}
/** @brief Stopping expansion covers a missing region without being applied twice. */
void StoppingFootprint() {
    const auto ground=Ground(Geometry({Cloud(Surface(0,0,0,true))}));
    auto query=Query({-.3,0,0},{-.3,0,0},.05);
    Require(Check(ground,query).status==s::GeometryStatus::kClear,"small stationary footprint remains supported");
    query.expanded_body.horizontal_radius=.4; query.stopping_distance=.35;
    Require(Check(ground,query).status!=s::GeometryStatus::kClear,"full braking footprint finds missing floor");
    query=Query({-.45,0,0},{-.45,0,0},.18); query.stopping_distance=.15;
    Require(Check(ground,query).status==s::GeometryStatus::kClear,"already expanded stopping radius is not added a second time");
}
/** @brief Snapshot equality by revision is insufficient; frame/epoch/calibration must match. */
void IdentityAndCalibration() {
    const auto packet=Cloud(Surface()); const auto geometry=Geometry({packet}), other=Geometry({packet});
    const auto ground=Ground(geometry);
    s::TrajectoryValidationOptions validation; validation.geometry_revision=17; validation.deadline=10.09;
    Require(!s::ValidateLocalTrajectory(Reference(),{3,10,100,4,10,100},validation,Safety(),other,ground,[]{return 10.05;}).validation.admitted,
        "equal-numbered distinct ESDF instances cannot reuse ground evidence");
    auto query=Query(); query.sweep.timing.alignment_epoch=10;
    Require(Check(ground,query).status!=s::GeometryStatus::kClear,"ground epoch mismatch denies");
    auto options=GroundOptions(); options.calibration.gravity_aligned=false;
    Require(!s::BuildLocalGround(geometry,options,[]{return 10.05;}).snapshot,"gravity is an explicit contract, never inferred");
    options=GroundOptions(); options.calibration.execution_frame="map";
    Require(!s::BuildLocalGround(geometry,options,[]{return 10.05;}).snapshot,"calibration frame mismatch denies");
}
/** @brief Calibration expiry cannot be extended by valid newer construction or older clocks. */
void ExpiryAndClock() {
    const auto geometry=Geometry({Cloud(Surface())}); auto options=GroundOptions(); options.calibration.valid_until=10.5;
    const auto ground=Ground(geometry,options);
    Require(!Admit(ground).validation.admitted,"support attestation must cover the entire executed reference");
    auto query=Query(); query.sweep.end_stamp=10.3;
    Require(Check(ground,query).status==s::GeometryStatus::kClear,"short interval stays inside attestation lifetime");
    Require(s::VerifyGroundSupport(*ground,query,[]{return 10.0;}).status!=s::GeometryStatus::kClear,"regressed ground-consumer time denies");
    Require(!s::BuildLocalGround(geometry,options,[]{return 10.6;}).snapshot,"rebuilding cannot renew expired calibration");
    Require(s::VerifyGroundSupport(*ground,query,[]{throw std::runtime_error("clock unavailable");return 10.05;}).status!=s::GeometryStatus::kClear,
        "clock dependency exception fails closed");
}
/** @brief One ground+body budget is shared over many individually affordable stationary chords. */
void AggregateWork() {
    const auto ground=Ground(Geometry({Cloud(Surface())}));
    const auto one=Admit(ground,Reference(.01,true));
    Require(one.validation.admitted,"one short stationary chord genuinely admits");
    auto safety=Safety();
    safety.maximum_work=one.examined_voxels+one.sweep_samples+one.support_queries+one.ground_work+1;
    Require(Admit(ground,Reference(.01,true),safety).validation.admitted,"one chord fits shared actual work allowance");
    Require(!Admit(ground,Reference(1,true),safety).validation.admitted,"multiple stationary chords cannot reset ground budget");
    auto query=Query(); query.maximum_work=1;
    const auto verdict=Check(ground,query);
    Require(verdict.status!=s::GeometryStatus::kClear && verdict.work==1,"ground scan honors remaining shared work and reports attempt");
}
/** @brief Invalid build limits, work exhaustion, deadlines and uncertainty reject evidence. */
void BoundsAndUncertainty() {
    const auto geometry=Geometry({Cloud(Surface())}); auto options=GroundOptions(); options.maximum_work=1;
    Require(!s::BuildLocalGround(geometry,options,[]{return 10.05;}).snapshot,"ground build cannot publish partially fitted model");
    options=GroundOptions(); options.deadline=10.04;
    Require(!s::BuildLocalGround(geometry,options,[]{return 10.05;}).snapshot,"expired build deadline denies");
    options=GroundOptions(); options.calibration.height_error=.2;
    Require(Check(Ground(geometry,options)).status!=s::GeometryStatus::kClear,"large calibrated uncertainty cannot pass tight floor tolerance");
    auto query=Query(); query.sweep.deadline=10.04;
    Require(Check(Ground(geometry),query).status!=s::GeometryStatus::kClear,"expired support deadline denies");
    query=Query(); query.reference_position_error=.2;
    Require(Check(Ground(geometry),query).status!=s::GeometryStatus::kClear,"reference tracking error is included in measured height compatibility");
}
/** @brief Real ground does not waive conservative occupied voxels near the robot body. */
void VoxelBoundaryConservatism() {
    auto options=GeometryOptions(); options.origin.z=-1;
    const auto result=s::BuildLocalEsdf({{Cloud(Surface()),Alignment()}},options,[]{return 10.05;});
    Require(static_cast<bool>(result.snapshot),"boundary-aligned measured floor builds valid geometry");
    const auto ground=Ground(result.snapshot);
    Require(Check(ground).status==s::GeometryStatus::kClear,"floor support exists independently of body clearance");
    Require(!Admit(ground).validation.admitted,"support never clears uncertain occupied floor voxels inside body envelope");
}
/** @brief Supported interior does not authorize an unknown exterior floor volume. */
void VolumeBoundary() {
    const auto ground=Ground(Geometry({Cloud(Surface())}));
    Require(Check(ground,Query({1.95,0,0},{1.95,0,0},.1)).status!=s::GeometryStatus::kClear,"stopping capsule touching volume exterior denies");
}
}  // namespace

/** @brief Run named independent production-consumer cases with nonzero failure status. */
int main() {
    const std::vector<std::pair<const char*,void(*)()>> cases={
        {"flat_three_modes",FlatThreeModes},{"drive_consumer",DifferentialDriveConsumer},
        {"shared_drive_body_ground_budget",SharedDriveBodyGroundBudget},
        {"provenance_memory",ProvenanceAndMemory},{"ramps_slope",Ramps},
        {"holes_between_endpoints",Holes},{"sparse_collinear",SparseAndCollinear},
        {"table_conflicting_heights",TabletopAndConflictingHeights},{"vertical_wall",Wall},
        {"steps_drops",StepsAndDrops},{"stopping_footprint",StoppingFootprint},
        {"identity_calibration",IdentityAndCalibration},{"expiry_clock",ExpiryAndClock},
        {"aggregate_ground_body_work",AggregateWork},{"bounds_uncertainty",BoundsAndUncertainty},
        {"volume_boundary",VolumeBoundary},{"voxel_boundary_conservatism",VoxelBoundaryConservatism}};
    try { for (const auto& item:cases) { item.second(); std::cout<<"PASS "<<item.first<<'\n'; } }
    catch (const std::exception& error) { std::cerr<<"FAIL "<<error.what()<<'\n'; return 1; }
    return 0;
}
