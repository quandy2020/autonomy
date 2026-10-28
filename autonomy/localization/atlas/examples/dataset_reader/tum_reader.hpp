#pragma once

#include <string>
#include <vector>

namespace autonomy::localization::atlas::examples {

struct TumRgbdSample {
    double color_timestamp = 0.0;
    double depth_timestamp = 0.0;
    std::string color_path;
    std::string depth_path;
};

class TumReader
{
public:
    static bool LoadAssociations(const std::string& dataset_root,
                                 const std::string& association_file,
                                 std::vector<TumRgbdSample>* samples);
};

}  // namespace autonomy::localization::atlas::examples
