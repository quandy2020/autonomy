/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/tasks/door_detector_task.hpp"

namespace
{

navigation::planning::DoorDetectorTask MakeDetector()
{
    return navigation::planning::DoorDetectorTask(
        navigation::planning::DoorDetectorOptions{0.4, 1.5, 5.0, 0.2});
}

automsgs::msgs::sensor_msgs::Image MakeImage(std::uint32_t gap_start,
                                             std::uint32_t gap_length)
{
    automsgs::msgs::sensor_msgs::Image image;
    image.set_width(200);
    image.set_height(20);
    image.set_encoding("32FC1");
    std::vector<float> values(200 * 20, 1.0F);
    for (std::uint32_t y = 0; y < 20; ++y)
    {
        for (std::uint32_t x = gap_start; x < gap_start + gap_length; ++x)
        {
            values[static_cast<std::size_t>(y) * 200 + x] = 0.0F;
        }
    }
    image.set_data(reinterpret_cast<const char*>(values.data()),
                   static_cast<int>(values.size() * sizeof(float)));
    return image;
}

}  // namespace

int main()
{
    automsgs::msgs::sensor_msgs::CameraInfo camera_info;
    camera_info.add_k(100.0);
    camera_info.add_k(0.0);
    camera_info.add_k(100.0);

    const auto detector = MakeDetector();
    navigation::planning::DoorDetection detection;
    if (!detector.Detect(MakeImage(80, 50), camera_info, &detection) ||
        !detection.detected || detection.width < 0.4 || detection.width > 0.7)
    {
        std::cerr << "door detection failed\n";
        return 1;
    }
    if (detector.Detect(MakeImage(0, 0), camera_info, &detection))
    {
        std::cerr << "wall accepted as door\n";
        return 1;
    }
    return 0;
}
