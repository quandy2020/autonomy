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
    if (argc != 5) {
        AERROR << "usage: mono_euroc VOCAB CONFIG DATASET TIMESTAMPS"
               << std::endl;
        return 2;
    }
    std::vector<EurocImageSample> samples;
    if (!EurocReader::LoadImages(argv[3], argv[4], false, &samples))
        return 1;
    MapBuilder builder(
        SystemConfig{argv[1], argv[2], SensorMode::kMonocular, false});
    if (!builder.Start())
        return 1;
    for (const auto& sample : samples) {
        ImageFrame image{sample.timestamp, "cam0",
                         cv::imread(sample.left_path, 0)};
        if (!builder.TrackMonocular(image))
            AWARN << "frame rejected" << std::endl;
    }
    builder.Stop();
    return 0;
}
