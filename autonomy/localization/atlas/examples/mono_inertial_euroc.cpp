#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/euroc_reader.hpp"

int main(int argc, char** argv) {
    namespace atlas = autonomy::localization::atlas;
    if (argc != 6) {
        AERROR << "usage: mono_inertial_euroc VOCAB CONFIG DATASET TIMESTAMPS "
                  "IMU_CSV"
               << std::endl;
        return 2;
    }
    std::vector<atlas::examples::EurocImageSample> images;
    std::vector<atlas::ImuSample> imu;
    if (!atlas::examples::EurocReader::LoadImages(argv[3], argv[4], false,
                                                  &images) ||
        !atlas::examples::EurocReader::LoadImu(argv[5], &imu))
        return 1;
    atlas::MapBuilder builder(atlas::SystemConfig{
        argv[1], argv[2], atlas::SensorMode::kMonocularInertial, false});
    if (!builder.Start())
        return 1;
    std::size_t imu_index = 0;
    for (const auto& sample : images) {
        while (imu_index < imu.size() &&
               imu[imu_index].time <= sample.timestamp) {
            builder.AddImu(imu[imu_index++]);
        }
        atlas::ImageFrame image{sample.timestamp, "cam0",
                                cv::imread(sample.left_path, 0)};
        if (!builder.TrackMonocular(image))
            AWARN << "frame rejected" << std::endl;
    }
    builder.Stop();
    return 0;
}
