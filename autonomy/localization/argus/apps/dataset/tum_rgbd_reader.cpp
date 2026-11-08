/*
 * Copyright 2026 The Openbot Authors
 */
#include "apps/dataset/tum_rgbd_reader.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

namespace argus {
namespace datasets {
namespace {

bool ParseStampLine(const std::string& line, double* t, std::string* rel) {
    if (line.empty() || line[0] == '#') {
        return false;
    }
    std::istringstream ss(line);
    if (!(ss >> *t >> *rel)) {
        return false;
    }
    return true;
}

}  // namespace

bool TumRgbdReader::LoadStampList(
    const std::string& path,
    std::vector<std::pair<double, std::string>>* out) {
    if (out == nullptr) {
        return false;
    }
    out->clear();
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        double t = 0;
        std::string rel;
        if (!ParseStampLine(line, &t, &rel)) {
            continue;
        }
        out->emplace_back(t, rel);
    }
    return !out->empty();
}

bool TumRgbdReader::Open(const std::string& root) {
    root_ = root;
    frames_.clear();

    std::vector<std::pair<double, std::string>> rgb;
    std::vector<std::pair<double, std::string>> depth;
    if (!LoadStampList(root + "/rgb.txt", &rgb)) {
        std::cerr << "[TumRgbdReader] missing/empty rgb.txt under " << root
                  << "\n";
        return false;
    }
    const bool has_depth = LoadStampList(root + "/depth.txt", &depth);

    std::size_t j = 0;
    for (const auto& r : rgb) {
        Assoc a;
        a.t = r.first;
        a.rgb_rel = r.second;
        if (has_depth) {
            while (j + 1 < depth.size() && depth[j + 1].first <= r.first) {
                ++j;
            }
            std::size_t best = j;
            if (j + 1 < depth.size()) {
                const double d0 = std::abs(depth[j].first - r.first);
                const double d1 = std::abs(depth[j + 1].first - r.first);
                if (d1 < d0) {
                    best = j + 1;
                }
            }
            if (std::abs(depth[best].first - r.first) <= max_dt_sec_) {
                a.depth_rel = depth[best].second;
            }
        }
        frames_.push_back(std::move(a));
    }
    return !frames_.empty();
}

bool TumRgbdReader::Read(std::size_t index, TumRgbdFrame* out,
                         bool load_depth) const {
    if (out == nullptr || index >= frames_.size()) {
        return false;
    }
    const Assoc& a = frames_[index];
    out->index = static_cast<int>(index);
    out->timestamp = a.t;
    out->rgb_path = root_ + "/" + a.rgb_rel;
    out->depth_path = a.depth_rel.empty() ? "" : (root_ + "/" + a.depth_rel);
    out->rgb = cv::imread(out->rgb_path, cv::IMREAD_COLOR);
    if (out->rgb.empty()) {
        return false;
    }
    out->depth.release();
    if (load_depth && !out->depth_path.empty()) {
        out->depth = cv::imread(out->depth_path, cv::IMREAD_UNCHANGED);
    }
    return true;
}

}  // namespace datasets
}  // namespace argus
