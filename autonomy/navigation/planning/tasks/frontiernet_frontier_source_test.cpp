/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/planning/tasks/frontiernet_frontier_source.hpp"

namespace
{

bool Near(double lhs, double rhs, double tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}

int CheckRegion()
{
    const double normalizer = 10.0;
    const double threshold = 0.5;
    // d = 0.4 m -> frontier, d = 1.0 m -> not a frontier.
    const float near = static_cast<float>(-std::log(0.4 / normalizer));
    const float far = static_cast<float>(-std::log(1.0 / normalizer));
    const auto region =
        navigation::planning::RegionFromDf({near, far}, normalizer, threshold);
    if (region.size() != 2 || region[0] != 1 || region[1] != 0)
    {
        std::cerr << "region decoding is wrong\n";
        return 1;
    }
    return 0;
}

int CheckGain()
{
    const std::uint32_t classes = 11;
    const std::uint32_t size = 2;
    std::vector<float> masks(classes * size * size, -10.0F);
    const auto set = [&](std::uint32_t pixel, std::uint32_t label) {
        masks[static_cast<std::size_t>(label) * size * size + pixel] = 10.0F;
    };
    set(0, 0);   // no frontier
    set(1, 1);   // first gain bin
    set(2, 10);  // last gain bin
    set(3, 5);
    const auto gain = navigation::planning::GainFromMasks(masks, classes, size);
    if (gain.size() != 4)
    {
        std::cerr << "gain size mismatch\n";
        return 1;
    }
    if (gain[0] != 0.0F)
    {
        std::cerr << "label 0 must map to zero gain\n";
        return 1;
    }
    const double first = std::expm1(0.5 * (std::log1p(1.0) + std::log1p(50.0)));
    const double last = std::expm1(0.5 * (std::log1p(2500.0) + std::log1p(3500.0)));
    if (!Near(gain[1], first, 1e-3) || !Near(gain[2], last, 1e-2))
    {
        std::cerr << "gain bin centres are wrong\n";
        return 1;
    }
    if (!(gain[1] < gain[3] && gain[3] < gain[2]))
    {
        std::cerr << "gain ordering is wrong\n";
        return 1;
    }
    return 0;
}

int CheckProjection()
{
    navigation::planning::FrontierCameraModel camera;
    camera.fx = 100.0;
    camera.fy = 100.0;
    camera.cx = 112.5;
    camera.cy = 112.5;
    // Principal point pixel with 3 m depth -> 3 m straight ahead in the base frame.
    const auto pose = navigation::common::MakePose2D(1.0, 2.0, M_PI / 2.0);
    double x = 0.0;
    double y = 0.0;
    if (!navigation::planning::ProjectPixelToMap(112, 112, 3.0F, camera, pose, 0.1,
                                                8.0, &x, &y))
    {
        std::cerr << "projection rejected a valid pixel\n";
        return 1;
    }
    if (!Near(x, 1.0, 1e-6) || !Near(y, 5.0, 1e-6))
    {
        std::cerr << "projection is wrong: " << x << ", " << y << "\n";
        return 1;
    }
    // A pixel to the right of the centre must land to the right of the heading.
    double right_x = 0.0;
    double right_y = 0.0;
    if (!navigation::planning::ProjectPixelToMap(162, 112, 3.0F, camera,
                                                 navigation::common::MakePose2D(0, 0, 0),
                                                 0.1, 8.0, &right_x, &right_y))
    {
        std::cerr << "projection rejected a valid pixel\n";
        return 1;
    }
    if (!(right_x > 0.0) || !(right_y < 0.0))
    {
        std::cerr << "camera-to-base rotation is wrong\n";
        return 1;
    }
    if (navigation::planning::ProjectPixelToMap(112, 112, 0.01F, camera, pose, 0.1,
                                                8.0, &x, &y))
    {
        std::cerr << "projection accepted a depth below the minimum\n";
        return 1;
    }
    return 0;
}

}  // namespace

int main()
{
    if (CheckRegion() != 0)
    {
        return 1;
    }
    if (CheckGain() != 0)
    {
        return 1;
    }
    if (CheckProjection() != 0)
    {
        return 1;
    }
    return 0;
}
