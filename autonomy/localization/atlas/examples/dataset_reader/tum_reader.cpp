#include "examples/dataset_reader/tum_reader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace autonomy::localization::atlas::examples {
namespace {

std::string Join(const std::string& root, const std::string& relative) {
    return root.empty() || root.back() == '/' ? root + relative
                                              : root + '/' + relative;
}

}  // namespace

bool TumReader::LoadAssociations(const std::string& dataset_root,
                                 const std::string& association_file,
                                 std::vector<TumRgbdSample>* samples) {
    if (samples == nullptr)
        return false;
    std::ifstream input(association_file);
    if (!input)
        return false;
    std::vector<TumRgbdSample> parsed;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#')
            continue;
        TumRgbdSample sample;
        std::string color;
        std::string depth;
        std::istringstream row(line);
        row >> sample.color_timestamp >> color >> sample.depth_timestamp >>
            depth;
        if (!row || !std::isfinite(sample.color_timestamp) ||
            !std::isfinite(sample.depth_timestamp)) {
            return false;
        }
        sample.color_path = Join(dataset_root, color);
        sample.depth_path = Join(dataset_root, depth);
        parsed.push_back(std::move(sample));
    }
    *samples = std::move(parsed);
    return !samples->empty();
}

}  // namespace autonomy::localization::atlas::examples
