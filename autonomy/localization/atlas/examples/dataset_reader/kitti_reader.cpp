#include "examples/dataset_reader/kitti_reader.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "atlas/common/logging.hpp"

namespace autonomy::localization::atlas::examples {
namespace {

std::string Join(const std::string& root, const std::string& suffix) {
    return root.empty() || root.back() == '/' ? root + suffix
                                              : root + '/' + suffix;
}

bool FileExists(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    return input.good();
}

}  // namespace

bool KittiReader::LoadStereo(const std::string& sequence_root,
                             std::vector<KittiImageSample>* samples) {
    if (samples == nullptr) {
        return false;
    }
    std::ifstream times(Join(sequence_root, "times.txt"));
    if (!times) {
        AERROR << "failed to open KITTI timestamps: "
               << Join(sequence_root, "times.txt") << std::endl;
        return false;
    }

    std::vector<KittiImageSample> parsed;
    std::string line;
    std::size_t frame_index = 0;
    while (std::getline(times, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream row(line);
        double timestamp = 0.0;
        row >> timestamp;
        if (!row || !std::isfinite(timestamp)) {
            AERROR << "invalid KITTI timestamp at frame " << frame_index
                   << std::endl;
            return false;
        }

        std::ostringstream filename;
        filename << std::setfill('0') << std::setw(6) << frame_index << ".png";
        KittiImageSample sample;
        sample.frame_index = frame_index;
        sample.timestamp = timestamp;
        sample.left_path = Join(sequence_root, "image_0/" + filename.str());
        sample.right_path = Join(sequence_root, "image_1/" + filename.str());
        if (!FileExists(sample.left_path) || !FileExists(sample.right_path)) {
            AERROR << "missing KITTI stereo pair at frame " << frame_index
                   << std::endl;
            return false;
        }
        parsed.push_back(std::move(sample));
        ++frame_index;
    }

    *samples = std::move(parsed);
    return !samples->empty();
}

}  // namespace autonomy::localization::atlas::examples
