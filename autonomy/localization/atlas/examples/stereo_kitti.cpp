#include <chrono>
#include <cstddef>
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
    for (const auto& sample : samples) {
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
        total_tracking_seconds += std::chrono::duration<double>(
                                      std::chrono::steady_clock::now() - begin)
                                      .count();
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
