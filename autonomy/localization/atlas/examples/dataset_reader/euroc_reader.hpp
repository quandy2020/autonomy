#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "atlas/sensor/sensor_types.hpp"

namespace autonomy::localization::atlas::examples {

struct EurocImageSample {
    double timestamp = 0.0;
    std::string left_path;
    std::string right_path;
};

class EurocReader
{
public:
    /**
     * Parses image timestamps. When skipped_missing_samples is provided,
     * samples whose required image files are absent are skipped and counted.
     */
    static bool LoadImages(const std::string& dataset_root,
                           const std::string& timestamps_file, bool stereo,
                           std::vector<EurocImageSample>* samples,
                           std::size_t* skipped_missing_samples = nullptr);
    static bool LoadImu(const std::string& csv_path,
                        std::vector<ImuSample>* samples);
};

}  // namespace autonomy::localization::atlas::examples
