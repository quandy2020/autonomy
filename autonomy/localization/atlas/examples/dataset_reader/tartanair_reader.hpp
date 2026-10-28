#pragma once

#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "atlas/sensor/sensor_types.hpp"

namespace autonomy::localization::atlas::examples {

struct TartanAirRgbdSample {
    double timestamp = 0.0;
    std::string color_path;
    std::string depth_path;
};

struct TartanAirSequence {
    std::vector<TartanAirRgbdSample> frames;
    std::vector<ImuSample> imu;
};

class TartanAirReader
{
public:
    // Loads frame paths, camera timestamps, and body-frame IMU samples from a
    // TartanAir V2 Pxxx sequence root. Counts and timestamps are validated
    // strictly; depth matrices are decoded on demand by LoadDepth().
    static bool LoadSequence(const std::string& sequence_root,
                             TartanAirSequence* sequence);

    // Decodes official packed-float PNG depth or legacy float32 NPY depth.
    // Values outside the synthetic sensor's (0, 80] metre range are invalid
    // and returned as zero in the CV_32FC1 matrix.
    static bool LoadDepth(const std::string& path, cv::Mat* depth_meters);

    // Decodes a little-endian, C-order, two-dimensional float32 NPY map.
    // Kept for compatibility with converted TartanAir datasets.
    static bool LoadDepthNpy(const std::string& path, cv::Mat* depth_meters);
};

}  // namespace autonomy::localization::atlas::examples
