#include <chrono>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/tum_reader.hpp"

int main(int argc, char** argv) {
    namespace atlas = autonomy::localization::atlas;
    if (argc != 5 && argc != 6) {
        AERROR << "usage: rgbd_tum VOCAB CONFIG DATASET ASSOCIATIONS "
                  "[TRAJECTORY]"
               << std::endl;
        return 2;
    }

    std::vector<atlas::examples::TumRgbdSample> samples;
    if (!atlas::examples::TumReader::LoadAssociations(argv[3], argv[4],
                                                      &samples) ||
        samples.empty()) {
        AERROR << "no RGB-D samples were loaded" << std::endl;
        return 1;
    }

    atlas::MapBuilder builder(
        atlas::SystemConfig{argv[1], argv[2], atlas::SensorMode::kRgbd, false});
    if (!builder.Start())
        return 1;

    std::size_t accepted_frames = 0;
    std::size_t valid_poses = 0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples[index];
        atlas::ImageFrame color{sample.color_timestamp, "rgb",
                                cv::imread(sample.color_path, -1)};
        atlas::ImageFrame depth{sample.depth_timestamp, "depth",
                                cv::imread(sample.depth_path, -1)};

        const auto started_at = std::chrono::steady_clock::now();
        atlas::TrackingResult result;
        if (!builder.TrackRgbd(color, depth, &result)) {
            AWARN << "RGB-D pair rejected at " << sample.color_timestamp
                  << std::endl;
        } else {
            ++accepted_frames;
            if (result.valid) {
                ++valid_poses;
            }
        }

        if (index + 1 < samples.size()) {
            const double frame_period =
                samples[index + 1].color_timestamp - sample.color_timestamp;
            const double processing_time =
                std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              started_at)
                    .count();
            if (frame_period > processing_time && frame_period < 1.0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(
                    frame_period - processing_time));
            }
        }
    }

    if (argc == 6 && !builder.ShutdownAndSaveTumTrajectory(argv[5])) {
        AERROR << "failed to save final TUM trajectory: " << argv[5]
               << std::endl;
        return 1;
    }
    builder.Stop();
    AINFO << "TUM RGB-D summary: samples=" << samples.size()
          << ", accepted=" << accepted_frames << ", valid_poses=" << valid_poses
          << std::endl;
    if (valid_poses == 0) {
        AERROR << "TUM RGB-D test produced no valid pose" << std::endl;
        return 1;
    }
    return 0;
}
