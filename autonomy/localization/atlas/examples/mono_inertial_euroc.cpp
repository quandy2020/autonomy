#include <chrono>
#include <cstddef>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/euroc_reader.hpp"

int main(int argc, char** argv) {
    namespace atlas = autonomy::localization::atlas;
    if (argc != 6 && argc != 7) {
        AERROR << "usage: mono_inertial_euroc VOCAB CONFIG DATASET TIMESTAMPS "
                  "IMU_CSV [TRAJECTORY]"
               << std::endl;
        return 2;
    }

    std::vector<atlas::examples::EurocImageSample> samples;
    std::vector<atlas::ImuSample> imu_samples;
    std::size_t skipped_missing_samples = 0;
    if (!atlas::examples::EurocReader::LoadImages(argv[3], argv[4], false,
                                                  &samples,
                                                  &skipped_missing_samples)) {
        AERROR << "failed to load EuRoC monocular images" << std::endl;
        return 1;
    }
    if (skipped_missing_samples > 0) {
        AWARN << "skipped " << skipped_missing_samples
              << " EuRoC samples with missing images" << std::endl;
    }
    if (!atlas::examples::EurocReader::LoadImu(argv[5], &imu_samples)) {
        AERROR << "failed to load EuRoC IMU data" << std::endl;
        return 1;
    }
    if (samples.empty() || imu_samples.empty()) {
        AERROR << "EuRoC monocular-inertial data is empty" << std::endl;
        return 1;
    }

    atlas::MapBuilder builder(atlas::SystemConfig{
        argv[1], argv[2], atlas::SensorMode::kMonocularInertial, false});
    if (!builder.Start()) {
        return 1;
    }

    std::size_t imu_index = 0;
    while (imu_index < imu_samples.size() &&
           imu_samples[imu_index].time <= samples.front().timestamp) {
        ++imu_index;
    }
    if (imu_index > 0) {
        --imu_index;
    }

    std::size_t valid_frames = 0;
    std::size_t unreadable_frames = 0;
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
        const auto tracking_start = std::chrono::steady_clock::now();
        atlas::ImageFrame image{
            sample.timestamp, "cam0",
            cv::imread(sample.left_path, cv::IMREAD_GRAYSCALE)};
        if (image.pixels.empty()) {
            ++unreadable_frames;
            AWARN_EVERY(100) << "skipping unreadable monocular image at "
                             << sample.timestamp;
        } else {
            atlas::TrackingResult result;
            if (!builder.TrackMonocular(image, &result)) {
                AWARN << "monocular-inertial frame rejected" << std::endl;
            } else if (result.valid) {
                ++valid_frames;
            }
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

    if (valid_frames == 0) {
        AERROR << "Atlas EuRoC monocular-inertial produced no valid frames"
               << std::endl;
        builder.Stop();
        return 1;
    }

    AINFO << "Atlas EuRoC monocular-inertial valid frames: " << valid_frames
          << "/" << samples.size() << ", missing: "
          << skipped_missing_samples << ", unreadable: "
          << unreadable_frames << std::endl;
    if (argc == 7) {
        if (!builder.ShutdownAndSaveEurocTrajectory(argv[6])) {
            AERROR << "failed to save EuRoC trajectory: " << argv[6]
                   << std::endl;
            return 1;
        }
    } else {
        builder.Stop();
    }
    return 0;
}
