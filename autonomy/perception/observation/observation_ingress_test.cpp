/* Copyright 2026 The Openbot Authors. */
/** @file observation_ingress_test.cpp
 * @brief Actual byte inputs exercise decoding, localization and Session coverage.
 * @details External navigation/fusion replies are controlled; native readers
 * and physical robot operation are outside this test's validation scope.
 */
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "autonomy/perception/observation/observation_ingress.hpp"
#include "autonomy/task/exploration/exploration_test_support.hpp"

namespace p = autonomy::perception;
namespace e = autonomy::task::exploration;
namespace {
using exploration_test::Require;
/** @brief Encode scalar bits independently of the decoder's implementation. */
void Word(std::string& data, std::size_t offset, std::uint64_t value, unsigned width, bool big = false) {
    for (unsigned i = 0; i < width; ++i)
        data[offset+i] = static_cast<char>((value >> (8*(big ? width-1-i : i))) & 255);
}
/** @brief Write an IEEE FLOAT32 measurement in the desired wire byte order. */
void Float(std::string& data, std::size_t offset, float value, bool big = false) {
    std::uint32_t bits; std::memcpy(&bits, &value, sizeof(bits)); Word(data, offset, bits, 4, big);
}
/** @brief Explicit rectified calibration of the actual encoded raster. */
p::DepthCalibration Calibration(int width = 1, int height = 1) {
    return {{width,height,1,1,0,0,.1,8},"depth_optical",7,true};
}
/** @brief External proof of actual sensor/base localization, including health. */
p::AcquisitionLocalization Localization(p::SensorKind sensor, const std::string& frame, double stamp) {
    p::AcquisitionLocalization proof;
    proof.sensor_frame=frame; proof.world_frame="map"; proof.map_epoch=1; proof.calibration_revision=7;
    proof.support=p::Mask(sensor); proof.healthy=true; proof.valid_from=stamp-1; proof.valid_until=stamp+1;
    return proof;
}
/** @brief Two real XYZ/time records with an explicitly padded row. */
struct Cloud {
    std::string data=std::string(36,'\0');
    p::PointCloudBytes bytes{2,1,16,36,false,"lidar",1,
        {{"x",0,p::FieldScalar::kFloat32,1},{"y",4,p::FieldScalar::kFloat32,1},
         {"z",8,p::FieldScalar::kFloat32,1},{"time",12,p::FieldScalar::kFloat32,1}}, {}};
    Cloud() { Float(data,0,1); Float(data,16,1); Float(data,28,.1F); bytes.data=data; }
};

/** @brief Metric conversion, padding and invalid pixels affect measured evidence. */
void DepthUnitsAndMasks() {
    std::string data(12,'\0'); Word(data,0,1000,2,true); Word(data,6,2000,2,true);
    p::DepthImageBytes image{2,2,6,true,"16UC1","depth_optical",1,data};
    std::optional<p::ObservationPacket> packet;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly,{},Localization,
        [&](auto value,double){packet=std::move(value);return true;},[]{return 1.1;});
    Require(ingress.PushDepth(image,Calibration(2,2),1)==p::ObservationIngressResult::kAccepted,
        "padded big-endian image admitted");
    Require(packet->observation.rays.size()==2 && packet->observation.valid_fraction==.5 &&
        packet->depth_view->axial_depth[0]==1 && packet->depth_view->axial_depth[2]==2,
        "16UC1 scaled once; invalid pixels retain the denominator");
    std::string floats(8,'\0'); Float(floats,0,1.25F); Float(floats,4,std::numeric_limits<float>::quiet_NaN());
    image={2,1,8,false,"32FC1","depth_optical",1,floats};
    Require(ingress.PushDepth(image,Calibration(2),2)==p::ObservationIngressResult::kAccepted &&
        packet->depth_view->axial_depth[0]==1.25F && !packet->depth_view->valid[1],
        "32FC1 remains metric; NaN does not create a free ray");
    auto tampered=*packet; tampered.depth_view->world_from_camera.translation.x=1;
    Require(!tampered.Valid(1.1),"depth transform must match actual localization proof");
    tampered=*packet; tampered.observation.rays.front().origin.x=1;
    Require(!tampered.Valid(1.1), "depth ray origin cannot diverge from the acquisition pose");
}

/** @brief Geometry sampling preserves full-raster quality and the complete actual encoder depth view. */
void DepthRaySampling() {
    std::string data(64,'\0');
    for (unsigned i = 0; i < 16; ++i) Float(data,i*4,i == 15 ? std::numeric_limits<float>::quiet_NaN() : 1.f);
    p::DepthImageBytes image{4,4,16,false,"32FC1","depth_optical",1,data};
    p::SensorDecodingOptions options; options.rgbd_pixel_stride = 2;
    std::optional<p::ObservationPacket> packet;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly,options,Localization,
        [&](auto value,double) { packet = std::move(value); return true; },[] { return 1.1; });
    Require(ingress.PushDepth(image,Calibration(4,4),1) == p::ObservationIngressResult::kAccepted,
        "actual sampled ray packet admitted");
    Require(packet && packet->observation.rays.size() == 4 && packet->observation.valid_fraction == 15./16 &&
        packet->depth_view && packet->depth_view->axial_depth.size() == 16 && !packet->depth_view->valid[15],
        "ray work bound cannot shrink encoder depth or invent pixel quality");
    options.rgbd_pixel_stride = 0; bool rejected = false;
    try { p::ValidateSensorDecodingOptions(options); } catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected,"zero pixel stride is not a valid acquisition policy");
}
/** @brief Reject invalid layouts and policies before allocation or byte access. */
void LayoutAndPolicy() {
    std::string data(2,'\0'); p::DepthImageBytes image{1,1,4,false,"16UC1","depth_optical",1,data};
    Require(!p::DecodeDepthImage(image,Calibration(),{}),"truncated row rejected");
    auto options=p::SensorDecodingOptions{}; options.maximum_payload_bytes=1; image.row_step=2;
    Require(!p::DecodeDepthImage(image,Calibration(),options),"payload budget enforced");
    image.width=std::numeric_limits<std::uint32_t>::max();
    Require(!p::DecodeDepthImage(image,Calibration(),{}),"oversized raster rejected");
    options.uint16_depth_scale=std::numeric_limits<double>::quiet_NaN();
    bool threw=false;try{p::ValidateSensorDecodingOptions(options);}catch(const std::invalid_argument&){threw=true;}
    Require(threw,"nonfinite metric scale rejected");
    Cloud cloud; cloud.bytes.fields[1].offset=0;
    Require(!p::DecodePointCloud(cloud.bytes,{}),"overlapping coordinates rejected");
    cloud.bytes.fields[1].offset=4;cloud.bytes.fields.push_back(cloud.bytes.fields.front());
    Require(!p::DecodePointCloud(cloud.bytes,{}),"ambiguous coordinate fields rejected");
}

/** @brief Actual point acquisition time determines motion compensation. */
void CloudTiming() {
    Cloud cloud;std::optional<p::ObservationPacket> packet;
    p::ObservationIngress ingress(p::SensorMode::kLidarOnly,{},
        [](auto sensor,const auto& frame,double stamp){auto pose=Localization(sensor,frame,stamp);
            pose.world_from_sensor.translation.x=pose.world_from_base.translation.x=(stamp-1)*10;return pose;},
        [&](auto value,double){packet=std::move(value);return true;},[]{return 1.2;});
    Require(ingress.PushCloud(cloud.bytes,1)==p::ObservationIngressResult::kAccepted &&
        packet->acquisition_poses.size()==2 && std::abs(packet->observation.rays[1].Endpoint().x-2)<1e-5,
        "later point transformed with actual acquisition-time motion");
    auto incomplete = *packet;
    incomplete.acquisition_poses.clear();
    Require(!incomplete.Valid(1.2), "each actual rolling-scan ray needs acquisition pose evidence");
    incomplete = *packet;
    incomplete.acquisition_poses.erase(incomplete.acquisition_poses.begin());
    Require(!incomplete.Valid(1.2), "missing one beam proof must not inherit scan-end pose");
    auto shifted = *packet;
    shifted.observation.rays.front().origin.x += 1;
    Require(!shifted.Valid(1.2), "ray origin must match its actual acquisition sensor pose");
    auto ambiguous = *packet;
    auto conflict = ambiguous.acquisition_poses.back();
    conflict.localization.world_from_base.translation.x += .5;
    ambiguous.acquisition_poses.push_back(conflict);
    Require(!ambiguous.Valid(1.2), "same-time conflicting base poses cannot attest one sensor scan");
    auto repeated = *packet;
    repeated.acquisition_poses.push_back(repeated.acquisition_poses.back());
    Require(repeated.Valid(1.2), "identical same-time beam proofs remain valid");
    auto wrong_end = *packet;
    wrong_end.localization.world_from_base.translation.x += .5;
    Require(!wrong_end.Valid(1.2), "scan-end packet pose must agree with its beam proof");
    auto unordered = *packet;
    std::swap(unordered.acquisition_poses[0], unordered.acquisition_poses[1]);
    Require(!unordered.Valid(1.2), "an externally supplied packet must retain ordered beam proofs");
    Float(cloud.data,12,.1F); Float(cloud.data,28,0);
    Require(ingress.PushCloud(cloud.bytes,2)==p::ObservationIngressResult::kAccepted &&
        packet->Valid(1.2) && packet->acquisition_poses.front().stamp==1 &&
        packet->observation.rays.front().stamp > packet->observation.rays.back().stamp,
        "nonchronological point storage preserves rays while ordering acquisition proofs");
    cloud.bytes.fields.pop_back();
    Require(ingress.PushCloud(cloud.bytes,2)==p::ObservationIngressResult::kInvalidMessage,
        "rolling scan must not guess missing times");
    p::SensorDecodingOptions instantaneous;instantaneous.instantaneous_cloud=true;
    Require(p::DecodePointCloud(cloud.bytes,instantaneous).has_value(),"explicit simultaneous acquisition can omit time");
}

/** @brief Health, calibration, within-scan epoch and expiry all constrain evidence. */
void IdentityAndExpiry() {
    std::string data(2,'\0');Word(data,0,1000,2);
    p::DepthImageBytes image{1,1,2,false,"16UC1","depth_optical",1,data};
    auto proof=Localization(p::SensorKind::kRgbd,image.frame_id,1);double now=1.1;unsigned accepted=0;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly,{},[&](auto,const auto&,double){return proof;},
        [&](auto,double){++accepted;return true;},[&]{return now;});
    proof.healthy=false;
    Require(ingress.PushDepth(image,Calibration(),1)==p::ObservationIngressResult::kInvalidLocalization,
        "TF and message arrival cannot prove localization health");
    proof.healthy=true;proof.calibration_revision=8;
    Require(ingress.PushDepth(image,Calibration(),2)==p::ObservationIngressResult::kInvalidLocalization,
        "calibration mismatch rejected");
    proof.calibration_revision=7;now=1.6;
    Require(ingress.PushDepth(image,Calibration(),3)==p::ObservationIngressResult::kInvalidEvidence && !accepted,
        "decoding does not renew acquisition expiry");
    Cloud cloud;p::ObservationIngress changed(p::SensorMode::kLidarOnly,{},
        [](auto sensor,const auto& frame,double stamp){auto pose=Localization(sensor,frame,stamp);
            pose.map_epoch=stamp<1.05?1:2;return pose;},[](auto,double){return true;},[]{return 1.2;});
    Require(changed.PushCloud(cloud.bytes,1)==p::ObservationIngressResult::kInvalidLocalization,
        "alignment change inside scan rejects the packet");
}

/** @brief Three modes, source revocation and dependency failures remain distinct. */
void ModesCloseAndFailures() {
    std::string data(2,'\0');Word(data,0,1000,2);
    p::DepthImageBytes image{1,1,2,false,"16UC1","depth_optical",1,data};Cloud cloud;
    for(auto mode:{p::SensorMode::kRgbdOnly,p::SensorMode::kLidarOnly,p::SensorMode::kLidarRgbd}){
        p::ObservationIngress ingress(mode,{},Localization,[](auto,double){return true;},[]{return 1.2;});
        Require(ingress.PushDepth(image,Calibration(),1)==((p::Mask(mode)&2)?
            p::ObservationIngressResult::kAccepted:p::ObservationIngressResult::kWrongMode),"depth mode policy");
        Require(ingress.PushCloud(cloud.bytes,1)==((p::Mask(mode)&1)?
            p::ObservationIngressResult::kAccepted:p::ObservationIngressResult::kWrongMode),"cloud mode policy");
        ingress.Close();Require(ingress.PushDepth(image,Calibration(),2)==p::ObservationIngressResult::kClosed,"source revoked");
    }
    p::ObservationIngress rejected(p::SensorMode::kRgbdOnly,{},Localization,[](auto,double){return false;},[]{return 1.1;});
    Require(rejected.PushDepth(image,Calibration(),1)==p::ObservationIngressResult::kSinkRejected,"queue rejection is not success");
    p::ObservationIngress throwing(p::SensorMode::kRgbdOnly,{},Localization,
        [](auto,double)->bool{throw 1;},[]{return 1.1;});
    Require(throwing.PushDepth(image,Calibration(),1)==p::ObservationIngressResult::kDependencyFailure,"exception contained");
}

/** @brief Raw depth is applied by the Session owner and satisfies actual view/dwell. */
void ActualSessionDepth() {
    exploration_test::Fixture fixture({0},.05);fixture.Arrive(1.02);
    Require(!fixture.session->Snapshot().last_observed_goal_id,"arrival alone is not observation");
    std::string data(2,'\0');Word(data,0,1000,2);
    p::DepthImageBytes image{1,1,2,false,"16UC1","depth_optical",1.03,data};double now=1.03;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly,{},
        [&](auto sensor,const auto& frame,double stamp){auto pose=Localization(sensor,frame,stamp);pose.map_epoch=fixture.epoch;
            pose.world_from_base.translation={1,1,0};pose.world_from_sensor.translation={1,1,.5};
            pose.world_from_sensor.rotation={0,0,1,-1,0,0,0,-1,0};return pose;},
        [&](auto value,double time){return fixture.session->PushSensorPacket(std::move(value),time);},[&]{return now;});
    Require(ingress.PushDepth(image,Calibration(),++fixture.sequence)==p::ObservationIngressResult::kAccepted,"actual packet admitted");
    Require(fixture.session->RecentDepthViews(now).empty(),"producer does not mutate owner cache");
    fixture.session->Tick(now);auto views=fixture.session->RecentDepthViews(now);
    Require(views.size()==1 && views[0]->localization.calibration_revision==7 && views[0]->view.axial_depth[0]==1 &&
        !fixture.session->Snapshot().last_observed_goal_id,"provenance retained; early frame cannot replace dwell");
    fixture.session->Tick(1.1);
    Require(fixture.session->Snapshot().last_observed_goal_id==42,"actual geometry and pose fulfill observation intent");
    Require(fixture.session->RecentDepthViews(2).empty(),"expired depth unavailable for learned features");
}

/** @brief Queue delay/rejected input remain source loss rather than completed work. */
void DelayedInputLoss() {
    exploration_test::Fixture fixture({0},.05);fixture.Arrive(1.02);
    std::string data(2,'\0');Word(data,0,1000,2);
    p::DepthImageBytes image{1,1,2,false,"16UC1","depth_optical",1.03,data};
    p::SensorDecodingOptions options;options.valid_for=.02;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly,options,
        [&](auto sensor,const auto& frame,double stamp){auto pose=Localization(sensor,frame,stamp);pose.map_epoch=fixture.epoch;
            pose.world_from_base.translation={1,1,0};return pose;},
        [&](auto value,double time){return fixture.session->PushSensorPacket(std::move(value),time);},[]{return 1.03;});
    Require(ingress.PushDepth(image,Calibration(),++fixture.sequence)==p::ObservationIngressResult::kAccepted,"fresh input queued");
    auto snapshot=fixture.session->Tick(1.06);
    Require(snapshot.observation_losses.pending_sources && !snapshot.completion_confirmed &&
        fixture.session->RecentDepthViews(1.06).empty(),"expired queued packet records loss without depth/coverage");
    fixture.session->RejectSensorInput(p::SensorKind::kRgbd,++fixture.sequence);
    Require(fixture.session->Tick(1.07).observation_losses.pending_sources,"unadmitted ordinal has direct loss channel");
}
/** @brief A packet at the Start seed time must establish actual localization.
 * @details No independent PushPose or synthetic observation is provided. The
 * admitted acquisition proof replaces the untrusted startup seed without adding
 * traveled distance or granting readiness before the owner processes the packet.
 */
void FirstPacketSeedsLocalization() {
    exploration_test::Fixture fixture({0}, .05, {}, false, false, 10, 32,
        e::ExplorationDriver::kBehaviorTree, false, {}, false);
    Require(!fixture.session->Snapshot().motion_ready,
        "startup coordinates alone do not attest localization");
    std::string data(2, '\0');
    Word(data, 0, 1000, 2);
    p::DepthImageBytes image{1, 1, 2, false, "16UC1", "depth_optical", 1, data};
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly, {},
        [&](auto sensor, const auto& frame, double stamp) {
            auto proof = Localization(sensor, frame, stamp);
            proof.map_epoch = fixture.epoch;
            proof.world_from_base.translation = {1, 1, 0};
            proof.world_from_sensor.translation = {1, 1, .5};
            proof.world_from_sensor.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
            return proof;
        },
        [&](auto packet, double now) {
            return fixture.session->PushSensorPacket(std::move(packet), now);
        }, [] { return 1.02; });
    Require(ingress.PushDepth(image, Calibration(), 1) == p::ObservationIngressResult::kAccepted,
        "first acquisition packet is admitted at the startup timestamp");
    Require(!fixture.session->Snapshot().motion_ready,
        "producer admission alone cannot advance owner readiness");
    const auto snapshot = fixture.session->Tick(1.02);
    Require(snapshot.motion_ready && snapshot.sensors.ready,
        "equal-time first packet must refresh current localization readiness");
    Require(snapshot.traveled_distance == 0,
        "replacing an unattested seed is not measured robot travel");
}

/** @brief Retain the four newest acquisition views even when delivery is delayed.
 * @details Receiver ordinals increase, but an older acquisition may arrive later.
 * It must neither replace a newer feature window member nor move current pose
 * backward. The test inspects the real Session cache before model inference.
 */
void DepthWindowAcquisitionOrder() {
    exploration_test::Fixture fixture({0}, .05, {}, false, false, 10, 32,
        e::ExplorationDriver::kBehaviorTree, false, {}, false);
    std::string data(2, '\0');
    Word(data, 0, 1000, 2);
    double now = 1.45;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly, {},
        [&](auto sensor, const auto& frame, double stamp) {
            auto proof = Localization(sensor, frame, stamp);
            proof.map_epoch = fixture.epoch;
            proof.world_from_base.translation = {stamp, 1, 0};
            proof.world_from_sensor.translation = {stamp, 1, .5};
            proof.world_from_sensor.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
            return proof;
        },
        [&](auto packet, double time) {
            return fixture.session->PushSensorPacket(std::move(packet), time);
        }, [&] { return now; });
    std::uint64_t sequence = 0;
    for (double stamp : {1.0, 1.1, 1.2, 1.3, 1.4, 1.05}) {
        p::DepthImageBytes image{1, 1, 2, false, "16UC1", "depth_optical", stamp, data};
        Require(ingress.PushDepth(image, Calibration(), ++sequence) == p::ObservationIngressResult::kAccepted,
            "valid delayed frame is admitted with its original acquisition time");
        fixture.session->Tick(now);
    }
    const auto views = fixture.session->RecentDepthViews(now);
    Require(views.size() == 4, "raw depth window is bounded to four views");
    for (std::size_t i = 0; i < views.size(); ++i)
        Require(std::abs(views[i]->view.stamp - (1.1 + .1 * i)) < 1e-9,
            "feature window preserves the four newest acquisition stamps in order");
    Require(std::abs(fixture.session->Snapshot().traveled_distance - .4) < 1e-9,
        "delayed acquisition pose does not fabricate return travel");
    const auto loss = fixture.session->Snapshot().observation_losses;
    Require((loss.pending_sources & p::Mask(p::SensorKind::kRgbd)) &&
        loss.last_loss.sequence[1] == 6,
        "rejected older acquisition is recorded without waiting for another frame");
    Require(fixture.session->RecentDepthViews(2).empty(),
        "expired cached depth cannot regain inference eligibility");
}

/** @brief Duplicate ordinals are replay, while a new backward-time ordinal is loss. */
void ReplayedOrdinalAndTemporalLoss() {
    p::ObservationBuffer buffer;
    buffer.Reset(1);
    p::SensorObservation observation;
    observation.sensor = p::SensorKind::kRgbd;
    observation.sequence = observation.map_epoch = 1;
    observation.stamp = 1;
    observation.valid_until = 2;
    observation.valid_fraction = 1;
    observation.localization_valid = true;
    observation.rays.push_back({{}, {1, 0, 0}, 1, true, 1, 1});
    Require(buffer.Push(observation, 1.1), "first actual ordinal admitted");
    Require(!buffer.Push(observation, 1.1) && !buffer.LossSnapshot().pending_sources,
        "replayed admitted ordinal does not manufacture a missing frame");
    observation.sequence = 2;
    observation.map_epoch = 2;
    Require(!buffer.Push(observation, 1.1) && !buffer.LossSnapshot().pending_sources,
        "other alignment does not contaminate this session's source loss");
    observation.map_epoch = 1;
    observation.stamp = observation.rays.front().stamp = .9;
    Require(!buffer.Push(observation, 1.1) && buffer.LossSnapshot().last_loss.sequence[1] == 2,
        "new rejected acquisition ordinal is immediately audited");
}

/** @brief A saturated owner event queue still reports the rejected source ordinal. */
void PacketQueueCapacityLoss() {
    exploration_test::Fixture fixture({0}, .05, {}, false, false, 10, 32,
        e::ExplorationDriver::kBehaviorTree, false, {}, false);
    std::string data(2, '\0');
    Word(data, 0, 1000, 2);
    p::DepthImageBytes image{1, 1, 2, false, "16UC1", "depth_optical", 1, data};
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly, {},
        [&](auto sensor, const auto& frame, double stamp) {
            auto proof = Localization(sensor, frame, stamp);
            proof.map_epoch = fixture.epoch;
            proof.world_from_base.translation = {1, 1, 0};
            proof.world_from_sensor.translation = {1, 1, .5};
            return proof;
        }, [&](auto packet, double now) {
            return fixture.session->PushSensorPacket(std::move(packet), now);
        }, [] { return 1.02; });
    for (std::uint64_t ordinal = 1; ordinal <= 128; ++ordinal)
        Require(ingress.PushDepth(image, Calibration(), ordinal) == p::ObservationIngressResult::kAccepted,
            "bounded owner queue accepts its available packet slots");
    Require(fixture.session->RecentDepthViews(1.02).empty(),
        "queued callbacks do not directly mutate the depth cache");
    Require(ingress.PushDepth(image, Calibration(), 129) == p::ObservationIngressResult::kSinkRejected,
        "full event queue rejects the complete packet");
    const auto snapshot = fixture.session->Tick(1.02);
    Require(snapshot.observation_losses.last_loss.sequence[1] == 129 &&
        snapshot.observation_losses.pending_sources && !snapshot.completion_confirmed,
        "capacity loss survives outside ordinary event slots and blocks completion");
}

/** @brief Early valid rays remain usable after a scan exceeds pose-history capacity.
 * @details Only the first twenty beams see the required surface at the required
 * yaw. The other beams use a different actual base orientation; the final beam
 * returns to the requested yaw but cannot see the surface. Source geometry is
 * encoded as actual XYZ/time bytes, and no extra synthetic pose is queued.
 */
void ExerciseLongScanProofs(bool deferred, bool expire_after_prefix) {
    exploration_test::Fixture fixture({0}, .05, {}, false, true, 10, 32,
        e::ExplorationDriver::kAutomatic, false, [deferred](auto& options) {
            options.mode = p::SensorMode::kLidarRgbd;
            options.validated_controller_modes = {1, 2, 3};
            if (deferred) options.maximum_observations_per_tick = 1;
        });
    fixture.Arrive(1.02);
    constexpr unsigned count = 600;
    std::string data(count * 16, '\0');
    for (unsigned index = 0; index < count; ++index) {
        Float(data, index * 16, index < 20 ? 1.F : .2F);
        Float(data, index * 16 + 12, .2F * index / count);
    }
    p::PointCloudBytes cloud{count, 1, 16, count * 16, false, "lidar", 1.05,
        {{"x", 0, p::FieldScalar::kFloat32, 1}, {"y", 4, p::FieldScalar::kFloat32, 1},
         {"z", 8, p::FieldScalar::kFloat32, 1}, {"time", 12, p::FieldScalar::kFloat32, 1}}, data};
    double now = 1.26;
    p::SensorDecodingOptions decoding;
    if (expire_after_prefix) decoding.valid_for = .02;
    p::ObservationIngress ingress(p::SensorMode::kLidarRgbd, decoding,
        [&](auto sensor, const auto& frame, double stamp) {
            auto proof = Localization(sensor, frame, stamp);
            proof.map_epoch = fixture.epoch;
            proof.support = p::Mask(p::SensorMode::kLidarRgbd);
            proof.world_from_base.translation = {1, 1, 0};
            if (stamp - cloud.stamp >= .007 && stamp - cloud.stamp < .199)
                proof.world_from_base.rotation = {0, -1, 0, 1, 0, 0, 0, 0, 1};
            proof.world_from_sensor = proof.world_from_base;
            proof.world_from_sensor.translation.z = .5;
            return proof;
        }, [&](auto packet, double now) {
            if (packet.observation.sensor == p::SensorKind::kLidar)
                Require(packet.acquisition_poses.size() == count, "all actual beam proofs enter the owner");
            return fixture.session->PushSensorPacket(std::move(packet), now);
        }, [&] { return now; });
    if (deferred) {
        std::string depth(2, '\0'); Word(depth, 0, 1000, 2);
        p::DepthImageBytes image{1, 1, 2, false, "16UC1", "depth_optical", 1.25, depth};
        Require(ingress.PushDepth(image, Calibration(), ++fixture.sequence) == p::ObservationIngressResult::kAccepted,
            "unrelated actual depth input occupies the first observation drain slot");
    }
    Require(ingress.PushCloud(cloud, 1) == p::ObservationIngressResult::kAccepted,
        "long scan is valid at actual consumption time");
    auto snapshot = fixture.session->Tick(now);
    Require(!snapshot.observation_losses.pending_sources && snapshot.sensors.ready,
        "valid long scan keeps source readiness and does not create input loss");
    if (deferred) {
        Require(!snapshot.last_observed_goal_id,
            "the examined depth prefix cannot satisfy the unseen LiDAR surface");
        now = 1.3;
        snapshot = fixture.session->Tick(now);
    }
    if (expire_after_prefix) {
        Require(!snapshot.last_observed_goal_id && snapshot.observation_losses.pending_sources &&
            !snapshot.completion_confirmed,
            "retained packet poses cannot renew an expired queued scan or confirm its view");
        return;
    }
    Require(snapshot.last_observed_goal_id == 42 && snapshot.observed_view_count == 1,
        "early actual rays must realize the required view beyond global pose-history capacity");
}

/** @brief Qualify all actual early beams independently of rolling pose capacity. */
void LongScanKeepsAcquisitionProofs() { ExerciseLongScanProofs(false, false); }
/** @brief A drained prefix cannot discard the proof for another source's suffix. */
void QueuedLongScanKeepsAcquisitionProofs() { ExerciseLongScanProofs(true, false); }
/** @brief Pose retention never renews the validity of a deferred scan. */
void QueuedLongScanExpires() { ExerciseLongScanProofs(true, true); }
}  // namespace
int main(){
    const std::pair<const char*,void(*)()> tests[]={
        {"depth units masks",DepthUnitsAndMasks},{"layout policy",LayoutAndPolicy},{"cloud timing",CloudTiming},
        {"depth ray sampling",DepthRaySampling},
        {"identity expiry",IdentityAndExpiry},{"modes close failures",ModesCloseAndFailures},
        {"actual Session depth",ActualSessionDepth},{"delayed loss",DelayedInputLoss},
        {"first packet localization", FirstPacketSeedsLocalization},
        {"depth window acquisition order", DepthWindowAcquisitionOrder},
        {"replay and temporal loss", ReplayedOrdinalAndTemporalLoss},
        {"packet queue capacity loss", PacketQueueCapacityLoss},
        {"long scan acquisition proofs", LongScanKeepsAcquisitionProofs},
        {"queued scan acquisition proofs", QueuedLongScanKeepsAcquisitionProofs},
        {"queued scan expiry", QueuedLongScanExpires}};
    for(const auto& test:tests){try{test.second();std::cout<<"PASS "<<test.first<<'\n';}
        catch(const std::exception& error){std::cerr<<"FAIL "<<test.first<<": "<<error.what()<<'\n';return 1;}}
}
