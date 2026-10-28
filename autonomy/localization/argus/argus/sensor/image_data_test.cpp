#include "sensor/image_data.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

namespace argus {
namespace sensor {
namespace {

TEST(ImageData, DecodesJpegIntoBgr) {
  cv::Mat raw(2, 3, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<std::uint8_t> encoded;
  ASSERT_TRUE(cv::imencode(".jpg", raw, encoded));

  CompressedImageData compressed;
  compressed.time = 1.5;
  compressed.frame_id = "cam0";
  compressed.format = "jpeg";
  compressed.data = encoded;

  ImageData image;
  ASSERT_TRUE(DecodeCompressedImage(compressed, &image));
  EXPECT_EQ(image.width, 3u);
  EXPECT_EQ(image.height, 2u);
  EXPECT_EQ(image.encoding, "bgr8");
  EXPECT_EQ(image.frame_id, "cam0");
  EXPECT_DOUBLE_EQ(image.time, 1.5);
  EXPECT_EQ(image.step, 9u);
  EXPECT_EQ(image.data.size(), 18u);
}

TEST(ImageData, RejectsEmptyCompressedBuffer) {
  ImageData image;
  image.width = 4;
  EXPECT_FALSE(DecodeCompressedImage(CompressedImageData{}, &image));
  EXPECT_EQ(image.width, 4u);
}

}  // namespace
}  // namespace sensor
}  // namespace argus
