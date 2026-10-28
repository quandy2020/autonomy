#include <chrono>
#include <cstddef>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/euroc_reader.hpp"

int main(int argc, char** argv) {
    namespace atlas = autonomy::localization::atlas;
    if (argc != 6 && argc != 7) {
        AERROR << "usage: stereo_inertial_tum_vi VOCAB CONFIG DATASET "
                  "TIMESTAMPS IMU_CSV [TRAJECTORY]"
               << std::endl;
        return 2;
    }

    std::vector<atlas::examples::EurocImageSample> samples;
    std::vector<atlas::ImuSample> imu_samples;
    if (!atlas::examples::EurocReader::LoadImages(argv[3], argv[4], true,
                                                  &samples) ||
        !atlas::examples::EurocReader::LoadImu(argv[5], &imu_samples)) {
        AERROR << "failed to load TUM-VI stereo-inertial data" << std::endl;
        return 1;
    }
    if (samples.empty() || imu_samples.empty()) {
        AERROR << "TUM-VI stereo-inertial data is empty" << std::endl;
        return 1;
    }

    atlas::MapBuilder builder(atlas::SystemConfig{
        argv[1], argv[2], atlas::SensorMode::kStereoInertial, false});
    if (!builder.Start()) {
        return 1;
    }

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    std::size_t imu_index = 0;
    while (imu_index < imu_samples.size() &&
           imu_samples[imu_index].time <= samples.front().timestamp) {
        ++imu_index;
    }
    if (imu_index > 0) {
        --imu_index;
    }

    std::size_t valid_frames = 0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples[index];
        if (index > 0) {
            while (imu_index < imu_samples.size() &&
                   imu_samples[imu_index].time <= sample.timestamp) {
                if (!builder.AddImu(imu_samples[imu_index])) {
                    AERROR << "rejected IMU sample at "
                           << imu_samples[imu_index].time << std::endl;
                    return 1;
                }
                ++imu_index;
            }
        }

        atlas::ImageFrame left{
            sample.timestamp, "cam0",
            cv::imread(sample.left_path, cv::IMREAD_GRAYSCALE)};
        atlas::ImageFrame right{
            sample.timestamp, "cam1",
            cv::imread(sample.right_path, cv::IMREAD_GRAYSCALE)};
        if (left.pixels.empty() || right.pixels.empty()) {
            AERROR << "failed to load TUM-VI stereo pair at timestamp "
                   << sample.timestamp << std::endl;
            return 1;
        }
        clahe->apply(left.pixels, left.pixels);
        clahe->apply(right.pixels, right.pixels);

        const auto tracking_start = std::chrono::steady_clock::now();
        atlas::TrackingResult result;
        if (!builder.TrackStereo(left, right, &result)) {
            AWARN << "TUM-VI stereo-inertial pair rejected" << std::endl;
        } else if (result.valid) {
            ++valid_frames;
        }
        const std::chrono::duration<double> tracking_duration =
            std::chrono::steady_clock::now() - tracking_start;
        double frame_interval = 0.0;
        if (index + 1 < samples.size()) {
            frame_interval = samples[index + 1].timestamp - sample.timestamp;
        } else if (index > 0) {
            frame_interval = sample.timestamp - samples[index - 1].timestamp;
        }
        const double sleep_seconds = frame_interval - tracking_duration.count();
        if (sleep_seconds > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(sleep_seconds));
        }
    }

    AINFO << "Atlas TUM-VI stereo-inertial valid frames: " << valid_frames
          << "/" << samples.size() << std::endl;
    if (argc == 7) {
        if (!builder.ShutdownAndSaveEurocTrajectory(argv[6])) {
            AERROR << "failed to save TUM-VI trajectory: " << argv[6]
                   << std::endl;
            return 1;
        }
    } else {
        builder.Stop();
    }
    return 0;
}
