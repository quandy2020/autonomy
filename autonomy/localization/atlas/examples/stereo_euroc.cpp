#include <algorithm>
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
    if (argc != 5 && argc != 6) {
        AERROR << "usage: stereo_euroc VOCAB CONFIG DATASET TIMESTAMPS "
                  "[TRAJECTORY]"
               << std::endl;
        return 2;
    }
    std::vector<atlas::examples::EurocImageSample> samples;
    std::size_t skipped_missing_samples = 0;
    if (!atlas::examples::EurocReader::LoadImages(argv[3], argv[4], true,
                                                  &samples,
                                                  &skipped_missing_samples))
        return 1;
    if (skipped_missing_samples > 0) {
        AWARN << "skipped " << skipped_missing_samples
              << " EuRoC samples with missing stereo images" << std::endl;
    }
    atlas::MapBuilder builder(atlas::SystemConfig{
        argv[1], argv[2], atlas::SensorMode::kStereo, false});
    if (!builder.Start())
        return 1;
    std::size_t valid_frames = 0;
    std::size_t unreadable_frames = 0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples[index];
        const auto tracking_start = std::chrono::steady_clock::now();
        atlas::ImageFrame left{sample.timestamp, "cam0",
                               cv::imread(sample.left_path, 0)};
        atlas::ImageFrame right{sample.timestamp, "cam1",
                                cv::imread(sample.right_path, 0)};
        if (left.pixels.empty() || right.pixels.empty()) {
            ++unreadable_frames;
            AWARN_EVERY(100) << "skipping unreadable stereo pair at "
                             << sample.timestamp;
        } else {
            atlas::TrackingResult result;
            if (!builder.TrackStereo(left, right, &result)) {
                AWARN << "stereo pair rejected" << std::endl;
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
        AERROR << "Atlas EuRoC stereo produced no valid frames" << std::endl;
        builder.Stop();
        return 1;
    }
    AINFO << "Atlas EuRoC stereo valid frames: " << valid_frames << "/"
          << samples.size() << ", missing: " << skipped_missing_samples
          << ", unreadable: " << unreadable_frames << std::endl;
    if (argc == 6) {
        if (!builder.ShutdownAndSaveEurocTrajectory(argv[5])) {
            AERROR << "failed to save EuRoC trajectory: " << argv[5]
                   << std::endl;
            return 1;
        }
    } else {
        builder.Stop();
    }
    return 0;
}
