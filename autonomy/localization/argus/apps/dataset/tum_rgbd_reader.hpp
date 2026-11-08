/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_APPS_DATASET_TUM_RGBD_READER_HPP_
#define ARGUS_APPS_DATASET_TUM_RGBD_READER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "apps/dataset/dataset_reader.hpp"

namespace argus {
namespace datasets {

/** @brief One TUM RGB-D frame (rgb + optional depth). */
struct TumRgbdFrame {
    int index = 0;
    double timestamp = 0.0;
    cv::Mat rgb;    /**< @brief BGR image. */
    cv::Mat depth;  /**< @brief Depth (16UC1 mm or 32FC1 m); may be empty. */
    std::string rgb_path;
    std::string depth_path;
};

/**
 * @brief Reader for TUM RGB-D freiburg sequences.
 *
 * Expects:
 *   <seq>/rgb.txt  depth.txt  (optional groundtruth.txt)
 *   <seq>/rgb/*.png  depth/*.png
 *
 * Association: nearest depth within @c max_dt_sec of each rgb stamp.
 */
class TumRgbdReader : public DatasetReader {
public:
    bool Open(const std::string& root) override;
    std::size_t size() const override { return frames_.size(); }
    bool empty() const override { return frames_.empty(); }
    const std::string& root() const override { return root_; }

    double max_dt_sec() const { return max_dt_sec_; }
    void set_max_dt_sec(double dt) { max_dt_sec_ = dt; }

    bool Read(std::size_t index, TumRgbdFrame* out, bool load_depth = true) const;

private:
    struct Assoc {
        double t = 0.0;
        std::string rgb_rel;
        std::string depth_rel;
    };

    static bool LoadStampList(const std::string& path,
                              std::vector<std::pair<double, std::string>>* out);

    std::string root_;
    double max_dt_sec_ = 0.02;
    std::vector<Assoc> frames_;
};

}  // namespace datasets
}  // namespace argus

#endif
