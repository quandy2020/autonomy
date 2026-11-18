/* Copyright 2026 The Openbot Authors. */
/** @file native_observation_mapper_test.cpp
 * @brief Verify existing-map fusion, exact generation prefixes and real native persistence.
 * @details Uses actual point-record decoding, GroundClassifier, ESDF, grid_map
 * and generated automsgs. Controlled localization does not replace a SLAM run.
 */
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include "autonomy/task/exploration/integration/native_observation_mapper.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace n = opt::navigation;

/** @brief Fail with the actual fusion/persistence invariant. */
void Require(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
/** @brief Encode real little-endian native point fields, independent from host byte order. */
void Float(std::string& bytes, float value) {
    std::uint32_t word = 0; std::memcpy(&word,&value,4);
    for (unsigned i = 0; i < 4; ++i) bytes.push_back(static_cast<char>((word>>(8*i))&255));
}
/** @brief Decode a dense actual hit-only floor scan with exact acquisition pose proofs. */
p::ObservationPacket Packet(std::uint64_t sequence, std::uint64_t epoch = 1) {
    std::string bytes;
    for (int x = -8; x <= 8; ++x) for (int y = -8; y <= 8; ++y) {
        Float(bytes,x*.2f); Float(bytes,y*.2f); Float(bytes,-1.f);
    }
    p::PointCloudBytes cloud{289,1,12,289*12,false,"lidar",10,
        {{"x",0,p::FieldScalar::kFloat32,1},{"y",4,p::FieldScalar::kFloat32,1},{"z",8,p::FieldScalar::kFloat32,1}},bytes};
    p::SensorDecodingOptions options; options.instantaneous_cloud = true; options.valid_for = 2;
    p::ObservationPacket packet;
    p::ObservationIngress ingress(p::SensorMode::kLidarOnly,options,
        [epoch](p::SensorKind sensor,const std::string& frame,double stamp) {
            p::AcquisitionLocalization proof;
            proof.world_from_sensor.translation.z = 1; proof.world_frame = "map"; proof.sensor_frame = frame;
            proof.map_epoch = epoch; proof.calibration_revision = 9; proof.support = p::Mask(sensor); proof.healthy = true;
            proof.valid_from = stamp; proof.valid_until = stamp+2;
            return std::optional<p::AcquisitionLocalization>{proof};
        },[&](auto value,double) { packet = std::move(value); return true; },[] { return 10.01; });
    Require(ingress.PushCloud(cloud,sequence) == p::ObservationIngressResult::kAccepted && packet.Valid(10.01),
        "actual point records decoded into complete acquisition-correlated packets");
    return packet;
}
/** @brief Allocate a small real mapping scope and explicit actual installation height. */
e::NativeObservationMapperOptions Options(const std::filesystem::path& directory) {
    e::NativeObservationMapperOptions options;
    options.mode = p::SensorMode::kLidarOnly; options.frame_id = "map";
    options.region = {-2,-2,2,2}; options.resolution = .2;
    options.minimum_height = -.2; options.maximum_height = 2.2;
    options.maximum_voxels = 10000; options.sensor_heights[0] = 1;
    options.output_directory = directory;
    return options;
}
/** @brief Explicit finite source requirement for the current actual geometry. */
e::RequestVersion Version(const e::MapReadView& map, std::uint64_t request, std::uint64_t ordinal) {
    e::RequestVersion version;
    version.session_id = 41; version.request_id = request; version.map_epoch = map.map_epoch;
    version.geometry_revision = map.geometry_revision; version.valid_until = 11;
    version.observation_cut.Include(p::SensorKind::kLidar,ordinal);
    return version;
}
/** @brief Read actual persisted native messages; missing files and parse failures fail the test. */
template <typename Message>
Message Read(const std::filesystem::path& path) {
    std::ifstream input(path,std::ios::binary); Require(static_cast<bool>(input),"actual saved native map exists");
    std::string bytes{std::istreambuf_iterator<char>(input),{}}; Message message;
    Require(message.ParseFromString(bytes),"actual saved native protobuf parses"); return message;
}
/** @brief Fusion requires every specified ordinal, and another generation cannot inherit its receipt. */
void PrefixAndPersistence(const std::filesystem::path& directory) {
    e::NativeObservationMapper mapper(Options(directory),[] { return 10.01; });
    const auto bootstrap = mapper.Process(0,Packet(1),10.01);
    Require(bootstrap && !bootstrap->mapping_synchronized && !bootstrap->observation_cut.present,
        "real pre-start geometry never certifies a session source prefix");
    auto first = mapper.Process(7,Packet(1),10.01);
    auto third = mapper.Process(7,Packet(3),10.01);
    Require(first && third && third->observation_cut.sequence[0] == 1,"missing second observation blocks the third's prefix");
    auto services = mapper.Services(7); auto version = Version(*third,1,3);
    Require(services.request_map_refresh(version),"actual refresh operation invoked");
    auto reply = services.poll_map_refresh();
    Require(reply && reply->success && !reply->fusion_confirmed && reply->fused_cut.sequence[0] == 1,
        "actual refresh cannot fabricate fusion across a missing ordinal");
    const auto complete = mapper.Process(7,Packet(2),10.01);
    Require(complete && complete->observation_cut.sequence[0] == 3,"real missing input closes the exact prefix");
    const auto before = mapper.Occupancy(); const auto duplicate = mapper.Process(7,Packet(2),10.01);
    Require(before && duplicate && mapper.Occupancy()->SerializeAsString() == before->SerializeAsString(),
        "owner replay does not integrate a completed source ordinal twice");
    version = Version(*complete,2,3);
    Require(services.request_map_refresh(version),"complete-prefix refresh invoked"); reply = services.poll_map_refresh();
    Require(reply && reply->success && reply->fusion_confirmed && reply->fused_cut.Covers(version.observation_cut),
        "receipt follows actual complete-prefix map fusion");
    auto other = mapper.Services(8);
    Require(other.request_map_refresh(version),"different generation refresh invoked"); reply = other.poll_map_refresh();
    Require(reply && !reply->fusion_confirmed,"new generation cannot inherit another generation's fusion cut");
    version = Version(*complete,3,3);
    Require(services.save_map("actual_map",version),"actual persistence operation invoked"); reply = services.poll_map_save();
    Require(reply && reply->success && reply->fusion_confirmed,"native save follows actual file synchronization");
    const auto occupancy = Read<::automsgs::msgs::map_msgs::OccupancyGrid>(directory/"actual_map.occupancy.pb");
    const auto grid = Read<::automsgs::msgs::map_msgs::GridMap>(directory/"actual_map.grid.pb");
    Require(occupancy.data_size() == 400 && occupancy.header().frame_id() == "map" && grid.layers_size() == 4,
        "both persisted maps retain actual bounded geometry and native grid_map layers");
    version = Version(*complete,4,4);
    Require(services.save_map("missing_input",version),"incomplete save operation reports an actual result");
    reply = services.poll_map_save();
    Require(reply && !reply->success && !std::filesystem::exists(directory/"missing_input.grid.pb"),
        "save acceptance cannot fabricate missing fusion or a saved map");
    version = Version(*complete,5,3);
    Require(services.request_map_refresh(version),"cancel-targeted operation invoked");
    services.cancel_mapping(version);
    Require(!services.poll_map_refresh() && !services.request_map_refresh(version),
        "canceled version cannot be polled or resubmitted by a late callback");
    const auto rebased = mapper.Process(7,Packet(4,2),10.01);
    Require(rebased && rebased->map_epoch == 2 && !rebased->observation_cut.present,
        "real coordinate change clears old geometry's source receipt");
}
/** @brief 2D free cells do not become free robot columns where height was never measured. */
void UnknownHeight(const std::filesystem::path& directory) {
    auto options = Options(directory); options.robot_height = 1.4;
    e::NativeObservationMapper mapper(options,[] { return 10.01; });
    const auto mapped = mapper.Process(9,Packet(1),10.01); const auto occupancy = mapper.Occupancy();
    Require(mapped && occupancy,"actual tall-robot geometry produced");
    std::size_t planar_free = 0, volume_free = 0;
    for (int value : occupancy->data()) if (value == 0) ++planar_free;
    for (int x = 0; x < mapped->grid->width(); ++x) for (int y = 0; y < mapped->grid->height(); ++y)
        if (mapped->grid->At(x,y).state == n::CellState::kFree) ++volume_free;
    Require(planar_free > 0 && volume_free == 0,
        "ESDF's unobserved-free default cannot fake missing vertical clearance");
    Require(mapped->grid->At(19,19).state == n::CellState::kUnknown,"unmeasured map corner remains unknown");
}
}  // namespace

/** @brief Run real mapping and persistence checks in this test's own temporary directory. */
int main() {
    std::string pattern = "/tmp/autonomy-native-mapping-test-XXXXXX";
    char* made = ::mkdtemp(pattern.data()); if (!made) return 2;
    const std::filesystem::path root(made);
    try {
        PrefixAndPersistence(root/"prefix"); UnknownHeight(root/"height");
        std::filesystem::remove_all(root);
        std::cout << "PASS native_observation_mapper" << std::endl; return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL native_observation_mapper: " << error.what() << " evidence=" << root << std::endl; return 1;
    }
}
