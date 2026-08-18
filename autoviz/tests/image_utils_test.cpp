#include "autoviz/display/image_utils.hpp"

#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autoviz {
namespace display {
namespace {

using Image = automsgs::msgs::sensor_msgs::Image;

TEST(ImageUtils, Decodes32fc1DepthImageToPreview) {
  Image image;
  image.set_width(2);
  image.set_height(2);
  image.set_encoding("32FC1");
  const float depth[] = {1.0f, 2.0f, 4.0f, 0.0f};
  image.set_data(std::string(reinterpret_cast<const char*>(depth), sizeof(depth)));

  const QImage decoded = imageFromProto(image);
  ASSERT_FALSE(decoded.isNull());
  ASSERT_EQ(decoded.format(), QImage::Format_RGB32);
  // Foxglove Turbo: near is blue-ish, far is red-ish, invalid is black.
  EXPECT_GT(qBlue(decoded.pixel(0, 0)), qBlue(decoded.pixel(0, 1)));
  EXPECT_GT(qRed(decoded.pixel(0, 1)), qRed(decoded.pixel(0, 0)));
  EXPECT_EQ(qGray(decoded.pixel(1, 1)), 0);
}

TEST(ImageUtils, DecodesRgb8ImageWithRowStep) {
  Image image;
  image.set_width(4);
  image.set_height(2);
  image.set_encoding("rgb8");
  image.set_step(16);
  const uint8_t pixels[] = {
      10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 0, 0, 0, 0,
      1,  2,  3,  4,  5,  6,  7,  8,  9,  10,  11,  12, 0, 0, 0, 0,
  };
  image.set_data(std::string(reinterpret_cast<const char*>(pixels), sizeof(pixels)));

  const QImage decoded = imageFromProto(image);
  ASSERT_FALSE(decoded.isNull());
  ASSERT_EQ(decoded.format(), QImage::Format_RGB32);
  EXPECT_EQ(qRed(decoded.pixel(0, 0)), 10);
  EXPECT_EQ(qGreen(decoded.pixel(1, 0)), 50);
  EXPECT_EQ(qBlue(decoded.pixel(3, 1)), 12);
}

TEST(ImageUtils, DecodesCapturedAutosimPayloadsWhenPresent) {
  const auto load = [](const char* path) -> std::string {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      return {};
    }
    return std::string(std::istreambuf_iterator<char>(input), {});
  };
  const std::string rgb_bytes = load("/tmp/rgb.pb");
  const std::string depth_bytes = load("/tmp/depth.pb");
  if (rgb_bytes.empty() || depth_bytes.empty()) {
    GTEST_SKIP() << "Captured autosim payloads not available";
  }

  Image rgb;
  ASSERT_TRUE(rgb.ParseFromString(rgb_bytes));
  const QImage rgb_image = imageFromProto(rgb);
  ASSERT_FALSE(rgb_image.isNull());
  EXPECT_EQ(rgb_image.width(), 640);
  EXPECT_EQ(rgb_image.height(), 480);
  EXPECT_GT(qGray(rgb_image.pixel(100, 100)), 0);
  int rgb_black = 0;
  for (int y = 0; y < rgb_image.height(); ++y) {
    for (int x = 0; x < rgb_image.width(); ++x) {
      if (qGray(rgb_image.pixel(x, y)) == 0) {
        ++rgb_black;
      }
    }
  }
  EXPECT_LT(rgb_black, rgb_image.width() * rgb_image.height() / 20);

  Image depth;
  ASSERT_TRUE(depth.ParseFromString(depth_bytes));
  const QImage depth_image = imageFromProto(depth);
  ASSERT_FALSE(depth_image.isNull());
  EXPECT_EQ(depth_image.width(), 640);
  EXPECT_EQ(depth_image.height(), 480);
  EXPECT_GT(qGray(depth_image.pixel(100, 100)), 0);
  int depth_black = 0;
  for (int y = 0; y < depth_image.height(); ++y) {
    for (int x = 0; x < depth_image.width(); ++x) {
      if (qGray(depth_image.pixel(x, y)) == 0) {
        ++depth_black;
      }
    }
  }
  EXPECT_LT(depth_black, depth_image.width() * depth_image.height() / 20);
}

TEST(ImageUtils, Rgb8PreservesCornerPixels) {
  Image image;
  image.set_width(8);
  image.set_height(8);
  image.set_encoding("rgb8");
  image.set_step(24);
  std::string pixels(static_cast<size_t>(8 * 8 * 3), '\0');
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      const size_t index = (static_cast<size_t>(y) * 8 + static_cast<size_t>(x)) * 3;
      pixels[index] = static_cast<char>(x * 16 + 8);
      pixels[index + 1] = static_cast<char>(y * 16 + 8);
      pixels[index + 2] = static_cast<char>(200);
    }
  }
  image.set_data(pixels);

  const QImage decoded = imageFromProto(image);
  ASSERT_FALSE(decoded.isNull());
  ASSERT_EQ(decoded.format(), QImage::Format_RGB32);
  EXPECT_EQ(qRed(decoded.pixel(7, 0)), 7 * 16 + 8);
  EXPECT_EQ(qGreen(decoded.pixel(7, 0)), 8);
  EXPECT_EQ(qBlue(decoded.pixel(7, 0)), 200);
  EXPECT_EQ(qRed(decoded.pixel(7, 7)), 7 * 16 + 8);
  EXPECT_EQ(qGreen(decoded.pixel(7, 7)), 7 * 16 + 8);
}

TEST(ImageUtils, Decodes16uc1DepthImageToPreview) {
  Image image;
  image.set_width(2);
  image.set_height(2);
  image.set_encoding("16UC1");
  const uint16_t depth_mm[] = {500, 1000, 2500, 0};
  image.set_data(
      std::string(reinterpret_cast<const char*>(depth_mm), sizeof(depth_mm)));

  const QImage decoded = imageFromProto(image);
  ASSERT_FALSE(decoded.isNull());
  ASSERT_EQ(decoded.format(), QImage::Format_RGB32);
  EXPECT_GT(qBlue(decoded.pixel(0, 0)), qBlue(decoded.pixel(0, 1)));
  EXPECT_GT(qRed(decoded.pixel(0, 1)), qRed(decoded.pixel(0, 0)));
  EXPECT_EQ(qGray(decoded.pixel(1, 1)), 0);
}

}  // namespace
}  // namespace display
}  // namespace autoviz
