#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/euroc_reader.hpp"

int main(int argc, char** argv) {
    using autonomy::localization::atlas::ImageFrame;
    using autonomy::localization::atlas::MapBuilder;
    using autonomy::localization::atlas::SensorMode;
    using autonomy::localization::atlas::SystemConfig;
    using autonomy::localization::atlas::examples::EurocImageSample;
    using autonomy::localization::atlas::examples::EurocReader;
    if (argc != 5 && argc != 6) {
        AERROR << "usage: mono_euroc VOCAB CONFIG DATASET TIMESTAMPS "
                  "[TRAJECTORY]"
               << std::endl;
        return 2;
    }
    std::vector<EurocImageSample> samples;
    std::size_t skipped_missing_samples = 0;
    if (!EurocReader::LoadImages(argv[3], argv[4], false, &samples,
                                 &skipped_missing_samples))
        return 1;
    if (skipped_missing_samples > 0) {
        AWARN << "skipped " << skipped_missing_samples
              << " EuRoC samples with missing images" << std::endl;
    }
    MapBuilder builder(
        SystemConfig{argv[1], argv[2], SensorMode::kMonocular, false});
    if (!builder.Start())
        return 1;
    std::size_t valid_frames = 0;
    std::size_t unreadable_frames = 0;
    for (const auto& sample : samples) {
        ImageFrame image{sample.timestamp, "cam0",
                         cv::imread(sample.left_path, 0)};
        if (image.pixels.empty()) {
            ++unreadable_frames;
            AWARN_EVERY(100) << "skipping unreadable monocular image at "
                             << sample.timestamp;
            continue;
        }
        autonomy::localization::atlas::TrackingResult result;
        if (!builder.TrackMonocular(image, &result)) {
            AWARN << "frame rejected" << std::endl;
        } else if (result.valid) {
            ++valid_frames;
        }
    }
    if (valid_frames == 0) {
        AERROR << "Atlas EuRoC monocular produced no valid frames"
               << std::endl;
        builder.Stop();
        return 1;
    }
    AINFO << "Atlas EuRoC monocular valid frames: " << valid_frames << "/"
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
