#include "atlas/common/io/config_loader.hpp"
#include "atlas/system.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>

#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>

namespace autonomy::localization::atlas {
namespace {

class ScopedConfigFile
{
public:
    ScopedConfigFile() {
        const auto unique_id =
            std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = (std::filesystem::temp_directory_path() /
                 ("atlas_config_loader_test_" + std::to_string(unique_id) +
                  ".yaml"))
                    .string();
        output_.open(path_);
        if (!output_) {
            throw std::runtime_error("failed to create test configuration");
        }
        output_ << "%YAML:1.0\n---\n";
    }

    ~ScopedConfigFile() {
        output_.close();
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    std::ostream& output() {
        return output_;
    }

    const std::string& Close() {
        output_.close();
        return path_;
    }

private:
    std::string path_;
    std::ofstream output_;
};

void WriteCommonSettings(std::ostream* output,
                         const std::string& camera_model) {
    *output << "Camera.type: \"" << camera_model << "\"\n"
            << "Camera.height: 480\n"
            << "Camera.width: 640\n"
            << "Camera.newHeight: 480\n"
            << "Camera.newWidth: 640\n"
            << "Camera.fps: 20\n"
            << "Camera.RGB: 1\n"
            << "ORBextractor.nFeatures: 1000\n"
            << "ORBextractor.scaleFactor: 1.2\n"
            << "ORBextractor.nLevels: 8\n"
            << "ORBextractor.iniThFAST: 20\n"
            << "ORBextractor.minThFAST: 7\n"
            << "Viewer.KeyFrameSize: 0.05\n"
            << "Viewer.KeyFrameLineWidth: 1.0\n"
            << "Viewer.GraphLineWidth: 0.9\n"
            << "Viewer.PointSize: 2.0\n"
            << "Viewer.CameraSize: 0.08\n"
            << "Viewer.CameraLineWidth: 3.0\n"
            << "Viewer.ViewpointX: 0.0\n"
            << "Viewer.ViewpointY: -0.7\n"
            << "Viewer.ViewpointZ: -1.8\n"
            << "Viewer.ViewpointF: 500.0\n"
            << "Viewer.imageViewScale: 1.0\n"
            << "System.LoadAtlasFromFile: \"unused\"\n"
            << "System.SaveAtlasToFile: \"unused\"\n"
            << "System.thFarPoints: 0.0\n";
}

void WriteStereoSettings(std::ostream* output) {
    *output << "Stereo.T_c1_c2: !!opencv-matrix\n"
            << "   rows: 4\n"
            << "   cols: 4\n"
            << "   dt: f\n"
            << "   data: [ 1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0,\n"
            << "           0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 ]\n"
            << "Stereo.ThDepth: 40.0\n";
}

TEST(ConfigLoaderTest, PublicTypeIsAvailable) {
    EXPECT_TRUE(std::is_class_v<ConfigLoader>);
}

TEST(ConfigLoaderTest, KeepsAsymmetricPinholeDistortionCoefficients) {
    ScopedConfigFile config;
    std::ostream& output = config.output();
    WriteCommonSettings(&output, "PinHole");
    WriteStereoSettings(&output);

    output << "Camera1.fx: 500.0\n"
           << "Camera1.fy: 501.0\n"
           << "Camera1.cx: 320.0\n"
           << "Camera1.cy: 240.0\n"
           << "Camera1.k1: 0.1\n"
           << "Camera1.k2: 0.01\n"
           << "Camera1.p1: 0.001\n"
           << "Camera1.p2: 0.002\n"
           << "Camera1.k3: 0.0001\n"
           << "Camera2.fx: 510.0\n"
           << "Camera2.fy: 511.0\n"
           << "Camera2.cx: 321.0\n"
           << "Camera2.cy: 241.0\n"
           << "Camera2.k1: -0.2\n"
           << "Camera2.k2: -0.02\n"
           << "Camera2.p1: -0.003\n"
           << "Camera2.p2: -0.004\n"
           << "Camera2.k3: -0.0002\n";

    ConfigLoader settings(config.Close(), System::STEREO);
    const cv::Mat distortion = settings.camera2DistortionCoef();
    const std::array<float, 5> expected{-0.2F, -0.02F, -0.003F, -0.004F,
                                        -0.0002F};

    ASSERT_EQ(distortion.total(), expected.size());
    for (std::size_t index = 0; index < expected.size(); ++index) {
        EXPECT_FLOAT_EQ(distortion.at<float>(static_cast<int>(index), 0),
                        expected[index]);
    }
}

TEST(ConfigLoaderTest, KeepsAsymmetricKannalaBrandtCamera2Coefficients) {
    ScopedConfigFile config;
    std::ostream& output = config.output();
    WriteCommonSettings(&output, "KannalaBrandt8");
    WriteStereoSettings(&output);

    output << "Camera1.fx: 450.0\n"
           << "Camera1.fy: 451.0\n"
           << "Camera1.cx: 310.0\n"
           << "Camera1.cy: 230.0\n"
           << "Camera1.k1: 0.01\n"
           << "Camera1.k2: 0.02\n"
           << "Camera1.k3: 0.03\n"
           << "Camera1.k4: 0.04\n"
           << "Camera1.overlappingBegin: 10\n"
           << "Camera1.overlappingEnd: 620\n"
           << "Camera2.fx: 460.0\n"
           << "Camera2.fy: 461.0\n"
           << "Camera2.cx: 311.0\n"
           << "Camera2.cy: 231.0\n"
           << "Camera2.k1: 0.11\n"
           << "Camera2.k2: 0.12\n"
           << "Camera2.k3: 0.13\n"
           << "Camera2.k4: 0.14\n"
           << "Camera2.overlappingBegin: 20\n"
           << "Camera2.overlappingEnd: 610\n";

    ConfigLoader settings(config.Close(), System::STEREO);
    GeometricCamera* camera2 = settings.camera2();
    const std::array<float, 4> expected{0.11F, 0.12F, 0.13F, 0.14F};

    ASSERT_NE(camera2, nullptr);
    ASSERT_GE(camera2->size(), 8U);
    for (std::size_t index = 0; index < expected.size(); ++index) {
        EXPECT_FLOAT_EQ(camera2->getParameter(index + 4), expected[index]);
    }
}

}  // namespace
}  // namespace autonomy::localization::atlas
