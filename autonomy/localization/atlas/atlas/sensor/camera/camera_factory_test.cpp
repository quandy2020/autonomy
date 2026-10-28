#include "atlas/sensor/camera/camera_factory.hpp"

#include <gtest/gtest.h>

#include "atlas/sensor/camera/geometric_camera.hpp"

namespace autonomy::localization::atlas {
namespace {

TEST(CameraFactoryTest, BuildsPinholeCamera) {
    auto camera =
        CameraFactory::Create("Pinhole", {500.F, 500.F, 320.F, 240.F});
    ASSERT_NE(camera, nullptr);
    const Eigen::Vector2f pixel =
        camera->project(Eigen::Vector3f(0.F, 0.F, 1.F));
    EXPECT_NEAR(pixel.x(), 320.F, 1e-5F);
    EXPECT_NEAR(pixel.y(), 240.F, 1e-5F);
}

TEST(CameraFactoryTest, RejectsInvalidModelOrParameterCount) {
    EXPECT_EQ(CameraFactory::Create("Unknown", {}), nullptr);
    EXPECT_EQ(CameraFactory::Create("Pinhole", {1.F, 2.F}), nullptr);
}

}  // namespace
}  // namespace autonomy::localization::atlas
