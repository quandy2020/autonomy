#include <chrono>
#include <cstddef>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/kitti_reader.hpp"

int main(int argc, char** argv) {
    namespace atlas = autonomy::localization::atlas;
    if (argc != 4 && argc != 5) {
        AERROR << "usage: stereo_kitti VOCAB CONFIG SEQUENCE_ROOT "
                  "[TRAJECTORY]"
               << std::endl;
        return 2;
    }

    std::vector<atlas::examples::KittiImageSample> samples;
    if (!atlas::examples::KittiReader::LoadStereo(argv[3], &samples)) {
        return 1;
    }

    atlas::MapBuilder builder(atlas::SystemConfig{
        argv[1], argv[2], atlas::SensorMode::kStereo, false});
    if (!builder.Start()) {
        return 1;
    }

    std::size_t valid_frames = 0;
    double total_tracking_seconds = 0.0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples[index];
        atlas::ImageFrame left{
            sample.timestamp, "image_0",
            cv::imread(sample.left_path, cv::IMREAD_GRAYSCALE)};
        atlas::ImageFrame right{
            sample.timestamp, "image_1",
            cv::imread(sample.right_path, cv::IMREAD_GRAYSCALE)};
        if (left.pixels.empty() || right.pixels.empty()) {
            AERROR << "failed to load KITTI frame " << sample.frame_index
                   << std::endl;
            return 1;
        }

        const auto begin = std::chrono::steady_clock::now();
        atlas::TrackingResult result;
        if (!builder.TrackStereo(left, right, &result)) {
            AWARN << "KITTI stereo frame rejected: " << sample.frame_index
                  << std::endl;
        } else if (result.valid) {
            ++valid_frames;
        }
        const double tracking_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          begin)
                .count();
        total_tracking_seconds += tracking_seconds;

        double frame_interval = 0.0;
        if (index + 1 < samples.size()) {
            frame_interval = samples[index + 1].timestamp - sample.timestamp;
        } else if (index > 0) {
            frame_interval = sample.timestamp - samples[index - 1].timestamp;
        }
        const double sleep_seconds = frame_interval - tracking_seconds;
        if (sleep_seconds > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(sleep_seconds));
        }
    }

    const double mean_tracking_ms =
        samples.empty() ? 0.0
                        : 1000.0 * total_tracking_seconds / samples.size();
    AINFO << "Atlas KITTI stereo valid frames: " << valid_frames << "/"
          << samples.size() << ", mean tracking: " << mean_tracking_ms << " ms"
          << std::endl;

    if (argc == 5) {
        if (!builder.ShutdownAndSaveKittiTrajectory(argv[4])) {
            AERROR << "failed to save KITTI trajectory: " << argv[4]
                   << std::endl;
            return 1;
        }
    } else {
        builder.Stop();
    }
    return 0;
}
