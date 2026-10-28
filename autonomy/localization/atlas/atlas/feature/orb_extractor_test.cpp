#include "atlas/feature/orb_extractor.hpp"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

namespace autonomy::localization::atlas {
namespace {

TEST(OrbExtractorTest, EmptyTextureHasNoFeatures) {
    OrbExtractor extractor(500, 1.2F, 8, 20, 7);
    cv::Mat image = cv::Mat::zeros(240, 320, CV_8UC1);
    std::vector<cv::KeyPoint> keypoints;
    std::vector<int> lapping_area;
    cv::Mat descriptors;
    extractor(image, cv::Mat(), keypoints, descriptors, lapping_area);
    EXPECT_TRUE(keypoints.empty());
}

}  // namespace
}  // namespace autonomy::localization::atlas
