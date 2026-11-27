/* Copyright 2026 The Openbot Authors. */
/** @file native_rgbd_platform_loader_test.cpp
 * @brief Focused validation of shared options loaded by the native RGB-D
 * bridge.
 */
#include "autonomy/task/exploration/integration/native_rgbd_platform.hpp"

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace autonomy::task::exploration {
namespace {

/** @brief Fail the standalone regression with one diagnostic. */
void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

/** @brief Minimal real calibration files and YAML accepted by the loader. */
class NativeRgbdLoaderFixture
{
public:
    NativeRgbdLoaderFixture()
        : root_(std::filesystem::temp_directory_path() /
                ("autonomy_native_rgbd_loader_" +
                 std::to_string(static_cast<long long>(::getpid())))) {
        std::filesystem::create_directory(root_);
        WriteCalibration();
        WriteYaml();
    }

    ~NativeRgbdLoaderFixture() {
        std::error_code ignored;
        std::filesystem::remove_all(root_, ignored);
    }

    NativeRgbdLoaderFixture(const NativeRgbdLoaderFixture&) = delete;
    NativeRgbdLoaderFixture& operator=(const NativeRgbdLoaderFixture&) = delete;

    /** @brief Load one shared-options JSON object through the production path.
     */
    NativeRgbdPlatformOptions Load(const std::string& json) const {
        std::ofstream output(root_ / "exploration.json");
        output << json;
        output.close();
        return LoadNativeRgbdPlatformOptions(root_ / "platform.yaml");
    }

    /** @brief Require that one options object is rejected as invalid. */
    void ExpectInvalid(const std::string& json,
                       const std::string& description) const {
        try {
            (void)Load(json);
        } catch (const std::invalid_argument&) {
            return;
        }
        throw std::runtime_error("loader accepted " + description);
    }

private:
    void WriteCalibration() const {
        ::automsgs::msgs::sensor_msgs::CameraInfo camera;
        camera.mutable_header()->set_frame_id("depth_optical");
        camera.set_width(2);
        camera.set_height(2);
        for (double value : {2.0, 0.0, 0.5, 0.0, 2.0, 0.5, 0.0, 0.0, 1.0}) {
            camera.add_k(value);
        }
        for (double value :
             {2.0, 0.0, 0.5, 0.0, 0.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0}) {
            camera.add_p(value);
        }
        std::ofstream camera_file(root_ / "camera.pb", std::ios::binary);
        Require(camera.SerializeToOstream(&camera_file),
                "failed to serialize CameraInfo fixture");

        ::automsgs::msgs::tf2_msgs::TFMessage tf;
        auto* mount = tf.add_transforms();
        mount->mutable_header()->set_frame_id("base_link");
        mount->set_child_frame_id("depth_optical");
        mount->mutable_transform()->mutable_translation()->set_z(1.0);
        mount->mutable_transform()->mutable_rotation()->set_w(1.0);
        std::ofstream tf_file(root_ / "tf.pb", std::ios::binary);
        Require(tf.SerializeToOstream(&tf_file),
                "failed to serialize TF fixture");
    }

    void WriteYaml() const {
        std::ofstream output(root_ / "platform.yaml");
        output << R"yaml(clock_domain: system
camera_info_file: camera.pb
tf_static_file: tf.pb
exploration_options_json: exploration.json
depth_topic: /depth
camera_info_topic: /camera_info
calibration_revision: 1
minimum_depth: 0.2
maximum_depth: 5.0
ray_pixel_stride: 4
atlas_frame: atlas
navigation_frame: map
physical_base_frame: base_link
navigation_base_frame: navigation_base
atlas_odometry_topic: /atlas/odometry
atlas_status_topic: /atlas/status
route_service: /route
planner_id: vista
controller_id: controller
validated_sensor_mask: )yaml"
               << static_cast<unsigned int>(
                      perception::Mask(perception::SensorMode::kRgbdOnly))
               << R"yaml(
time_budget: 60.0
require_return: true
require_save: false
map_name: loader_test
region:
  min_x: -5.0
  min_y: -5.0
  max_x: 5.0
  max_y: 5.0
mapping_resolution: 0.25
mapping_maximum_voxels: 100000
robot_height: 0.5
vertical_margin: 0.05
robot_radius: 0.25
horizontal_margin: 0.05
map_output_directory: maps
base_odometry_topic: /base/odometry
occupancy_topic: /occupancy
grid_topic: /grid
host_vista_server: false
)yaml";
    }

    std::filesystem::path root_;
};

void SensorRangeValidation(const NativeRgbdLoaderFixture& fixture) {
    const auto narrowed =
        fixture.Load(R"json({"mapFrame":"map","sensorRangeM":3.0})json");
    Require(narrowed.exploration.input.maximum_depth == 3.0,
            "sensor range did not narrow input depth");
    Require(narrowed.exploration.camera.has_value() &&
                narrowed.exploration.camera->camera.max_depth == 3.0,
            "sensor range did not narrow camera visibility");

    fixture.ExpectInvalid(R"json({"mapFrame":"map","sensorRangeM":0.1})json",
                          "sensor range below calibrated minimum");
    fixture.ExpectInvalid(R"json({"mapFrame":"map","sensorRangeM":6.0})json",
                          "sensor range above calibrated maximum");
    fixture.ExpectInvalid(R"json({"mapFrame":"map","sensorRangeM":"NaN"})json",
                          "non-finite sensor range");
}

void UnsupportedPolicyValidation(const NativeRgbdLoaderFixture& fixture) {
    fixture.ExpectInvalid(R"json({"mapFrame":"map","farConnectDistM":1.0})json",
                          "unsupported FAR behavior");
    fixture.ExpectInvalid(R"json({"mapFrame":"map","usePriorMap":true})json",
                          "unsupported prior-map source");
    fixture.ExpectInvalid(
        R"json({"mapFrame":"map","usePlannerCostmap":true})json",
        "unsupported planner-costmap source");
}

void TerrainFieldValidation(const NativeRgbdLoaderFixture& fixture) {
    const auto options = fixture.Load(R"json({
      "mapFrame":"map",
      "useTerrainHeight":true,
      "viewpointNumX":4,
      "viewpointNumY":6,
      "viewpointNumZ":8,
      "viewpointResolutionM":0.5,
      "viewpointResolutionZM":0.25
    })json");
    Require(options.exploration.coverage.rolling_field.has_value(),
            "terrain policy did not configure a rolling field");
    const auto& field = *options.exploration.coverage.rolling_field;
    Require(field.dimensions[0] == 8 && field.dimensions[1] == 12 &&
                field.dimensions[2] == 8,
            "terrain field dimensions are not metric or anisotropic");
    Require(field.maximum_cells == 768,
            "terrain field cell bound does not match dimensions");
    Require(field.maximum_input_voxels == 100000,
            "terrain field did not inherit native input voxel cap");

    fixture.ExpectInvalid(R"json({
      "mapFrame":"map",
      "useTerrainHeight":true,
      "viewpointNumX":4,
      "viewpointNumY":4,
      "viewpointNumZ":4,
      "viewpointResolutionM":0.5,
      "viewpointResolutionZM":-0.1
    })json",
                          "negative terrain Z resolution");
    fixture.ExpectInvalid(R"json({
      "mapFrame":"map",
      "useTerrainHeight":true,
      "viewpointNumX":100,
      "viewpointNumY":100,
      "viewpointNumZ":100,
      "viewpointResolutionM":1.0,
      "viewpointResolutionZM":1.0
    })json",
                          "terrain field beyond native voxel cap");
}

}  // namespace
}  // namespace autonomy::task::exploration

int main() {
    try {
        autonomy::task::exploration::NativeRgbdLoaderFixture fixture;
        autonomy::task::exploration::SensorRangeValidation(fixture);
        autonomy::task::exploration::UnsupportedPolicyValidation(fixture);
        autonomy::task::exploration::TerrainFieldValidation(fixture);
        std::cout << "native_rgbd_platform_loader PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
