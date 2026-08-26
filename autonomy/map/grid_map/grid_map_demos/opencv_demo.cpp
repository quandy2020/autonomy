/*
 * opencv_demo.cpp — ROS-free OpenCV blur demo.
 * Usage: opencv_demo [frames=10] [out_dir] [--gui]
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_converter.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  int frames = 10;
  std::string out_dir = "/tmp/grid_map_demos/opencv";
  bool gui = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--gui") == 0) {
      gui = true;
    } else if (frames == 10 && argv[i][0] != '-') {
      frames = std::atoi(argv[i]);
    } else if (argv[i][0] != '-') {
      out_dir = argv[i];
    }
  }

  GridMap map({"original", "elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.01);
  map["original"].setZero();

  Polygon polygon;
  polygon.setFrameId(map.getFrameId());
  polygon.addVertex(Position(0.480, 0.000));
  polygon.addVertex(Position(0.164, 0.155));
  polygon.addVertex(Position(0.116, 0.500));
  polygon.addVertex(Position(-0.133, 0.250));
  polygon.addVertex(Position(-0.480, 0.399));
  polygon.addVertex(Position(-0.316, 0.000));
  polygon.addVertex(Position(-0.480, -0.399));
  polygon.addVertex(Position(-0.133, -0.250));
  polygon.addVertex(Position(0.116, -0.500));
  polygon.addVertex(Position(0.164, -0.155));
  polygon.addVertex(Position(0.480, 0.000));
  for (PolygonIterator iterator(map, polygon); !iterator.isPastEnd();
       ++iterator) {
    map.at("original", *iterator) = 0.3f;
  }

  cv::Mat originalImage;
  GridMapCvConverter::toImage<unsigned short, 1>(map, "original", CV_16UC1, 0.0,
                                                 0.3, originalImage);

  if (gui) {
    cv::namedWindow("OpenCV Demo");
  }

  for (int frame = 0; frame < frames; ++frame) {
    const double t = frame / 30.0;
    map.setTimestamp(static_cast<uint64_t>(t * 1e9));
    int blurRadius = 200 - std::abs(static_cast<int>(200.0 * std::sin(t)));
    blurRadius = blurRadius - (blurRadius % 2) + 1;

    cv::Mat modifiedImage;
    cv::GaussianBlur(originalImage, modifiedImage, cv::Size(blurRadius, blurRadius),
                     0.0, 0.0);
    if (gui) {
      cv::imshow("OpenCV Demo", modifiedImage);
      cv::waitKey(40);
    }
    GridMapCvConverter::addLayerFromImage<unsigned short, 1>(
        modifiedImage, "elevation", map, 0.0, 0.3);
    AINFO << "Frame " << frame << " blur radius " << blurRadius;
  }

  if (gui) {
    cv::destroyAllWindows();
  }
  grid_map::grid_map_demos::saveLayerAsPng(map, "elevation",
                                 std::filesystem::path(out_dir) / "elevation.png");
  grid_map::grid_map_demos::saveMapProto(map, std::filesystem::path(out_dir) / "map.pb");
  return 0;
}
