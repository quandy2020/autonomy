/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/map/costmap_2d/map_io.hpp"

#include <libgen.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "autonomy/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/map_mode.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/transform/tf2/LinearMath/Matrix3x3.h"
#include "autonomy/transform/tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"
namespace {
constexpr double kPixelMax = 255.0;

double ComputeShade(const cv::Mat& img, int x, int y,
                    autonomy::map::costmap_2d::MapMode mode,
                    bool* has_transparency = nullptr) {
    const int channels = img.channels();
    if (channels == 1) {
        return img.at<uint8_t>(y, x) / kPixelMax;
    }

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t alpha = 255;
    if (channels == 4) {
        const cv::Vec4b pixel = img.at<cv::Vec4b>(y, x);
        blue = pixel[0];
        green = pixel[1];
        red = pixel[2];
        alpha = pixel[3];
        if (has_transparency != nullptr) {
            *has_transparency = (alpha != 255);
        }
    } else {
        const cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);
        blue = pixel[0];
        green = pixel[1];
        red = pixel[2];
    }

    std::vector<double> values = {red / kPixelMax, green / kPixelMax,
                                  blue / kPixelMax};
    if (mode == autonomy::map::costmap_2d::MapMode::Trinary && channels == 4) {
        values.push_back((kPixelMax - alpha) / kPixelMax);
    }

    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}
}  // namespace

namespace autonomy {
namespace map {
namespace costmap_2d {

// === Map input part ===

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly
/// nicer error messages, including the name of the failed key
/// @throw YAML::Exception
template <typename T>
T yaml_get_value(const YAML::Node& node, const std::string& key) {
    try {
        return node[key].as<T>();
    } catch (YAML::Exception& e) {
        std::stringstream ss;
        ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
        throw YAML::Exception(e.mark, ss.str());
    }
}

std::string get_home_dir() {
    if (const char* home_dir = std::getenv("HOME")) {
        return std::string{home_dir};
    }
    return std::string{};
}

std::string expand_user_home_dir_if_needed(std::string yaml_filename,
                                           std::string home_variable_value) {
    if (yaml_filename.size() < 2 ||
        !(yaml_filename[0] == '~' && yaml_filename[1] == '/')) {
        return yaml_filename;
    }

    if (home_variable_value.empty()) {
        AINFO << "[map_io] Map yaml file name starts with '~/' but no HOME "
                 "variable set.";
        AINFO << "[map_io] User home dir will be not expanded.";
        return yaml_filename;
    }
    const std::string prefix{home_variable_value};
    return yaml_filename.replace(0, 1, prefix);
}

LoadParameters loadMapYaml(const std::string& yaml_filename) {
    YAML::Node doc = YAML::LoadFile(
        expand_user_home_dir_if_needed(yaml_filename, get_home_dir()));
    LoadParameters load_parameters;

    auto image_file_name = yaml_get_value<std::string>(doc, "image");
    if (image_file_name.empty()) {
        throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
    }
    if (image_file_name[0] != '/') {
        // dirname takes a mutable char *, so we copy into a vector
        std::vector<char> fname_copy(yaml_filename.begin(),
                                     yaml_filename.end());
        fname_copy.push_back('\0');
        image_file_name =
            std::string(dirname(fname_copy.data())) + '/' + image_file_name;
    }
    load_parameters.image_file_name = image_file_name;

    load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
    load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
    if (load_parameters.origin.size() != 3) {
        throw YAML::Exception(
            doc["origin"].Mark(),
            "value of the 'origin' tag should have 3 elements, not " +
                std::to_string(load_parameters.origin.size()));
    }

    load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
    load_parameters.occupied_thresh =
        yaml_get_value<double>(doc, "occupied_thresh");

    auto map_mode_node = doc["mode"];
    if (!map_mode_node.IsDefined()) {
        load_parameters.mode = MapMode::Trinary;
    } else {
        load_parameters.mode =
            map_mode_from_string(map_mode_node.as<std::string>());
    }

    try {
        load_parameters.negate = yaml_get_value<int>(doc, "negate");
    } catch (YAML::Exception&) {
        load_parameters.negate = yaml_get_value<bool>(doc, "negate");
    }

    AINFO << "[map_io] resolution: " << load_parameters.resolution;
    AINFO << "[map_io] origin[0]: " << load_parameters.origin[0];
    AINFO << "[map_io] origin[1]: " << load_parameters.origin[1];
    AINFO << "[map_io] origin[2]: " << load_parameters.origin[2];
    AINFO << "[map_io] free_thresh: " << load_parameters.free_thresh;
    AINFO << "[map_io] occupied_thresh: " << load_parameters.occupied_thresh;
    AINFO << "[map_io] mode: " << map_mode_to_string(load_parameters.mode);
    AINFO << "[map_io] negate: " << load_parameters.negate;

    return load_parameters;
}

void loadMapFromFile(const LoadParameters& load_parameters,
                     commsgs::map_msgs::OccupancyGrid& map) {
    commsgs::map_msgs::OccupancyGrid msg;

    AINFO << "[map_io] Loading image_file: " << load_parameters.image_file_name;
    cv::Mat img =
        cv::imread(load_parameters.image_file_name, cv::IMREAD_UNCHANGED);
    if (img.empty()) {
        throw std::runtime_error("Failed to load image: " +
                               load_parameters.image_file_name);
    }

    msg.info.width = static_cast<uint32_t>(img.cols);
    msg.info.height = static_cast<uint32_t>(img.rows);

    msg.info.resolution = load_parameters.resolution;
    msg.info.origin.position.x = load_parameters.origin[0];
    msg.info.origin.position.y = load_parameters.origin[1];
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation =
        utils::OrientationAroundZAxis(load_parameters.origin[2]);

    msg.data.resize(msg.info.width * msg.info.height);

    for (size_t y = 0; y < msg.info.height; y++) {
        const int src_y = static_cast<int>(y);
        for (size_t x = 0; x < msg.info.width; x++) {
            const int src_x = static_cast<int>(x);
            bool has_transparency = false;
            const double shade =
                ComputeShade(img, src_x, src_y, load_parameters.mode,
                             &has_transparency);

            const double occ =
                (load_parameters.negate ? shade : 1.0 - shade);

            int8_t map_cell;
            switch (load_parameters.mode) {
                case MapMode::Trinary:
                    if (load_parameters.occupied_thresh < occ) {
                        map_cell = utils::OCC_GRID_OCCUPIED;
                    } else if (occ < load_parameters.free_thresh) {
                        map_cell = utils::OCC_GRID_FREE;
                    } else {
                        map_cell = utils::OCC_GRID_UNKNOWN;
                    }
                    break;
                case MapMode::Scale:
                    if (has_transparency) {
                        map_cell = utils::OCC_GRID_UNKNOWN;
                    } else if (load_parameters.occupied_thresh < occ) {
                        map_cell = utils::OCC_GRID_OCCUPIED;
                    } else if (occ < load_parameters.free_thresh) {
                        map_cell = utils::OCC_GRID_FREE;
                    } else {
                        map_cell =
                            std::rint((occ - load_parameters.free_thresh) /
                                      (load_parameters.occupied_thresh -
                                       load_parameters.free_thresh) *
                                      100.0);
                    }
                    break;
                case MapMode::Raw: {
                    double occ_percent = std::round(shade * 255);
                    if (utils::OCC_GRID_FREE <= occ_percent &&
                        occ_percent <= utils::OCC_GRID_OCCUPIED) {
                        map_cell = static_cast<int8_t>(occ_percent);
                    } else {
                        map_cell = utils::OCC_GRID_UNKNOWN;
                    }
                    break;
                }
                default:
                    throw std::runtime_error("Invalid map mode");
            }
            msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
        }
    }

    msg.info.map_load_time = Time::Now();
    msg.header.frame_id = "map";
    msg.header.stamp = Time::Now();

    AERROR << "[map_io] Read map " << load_parameters.image_file_name << ": "
           << msg.info.width << " X " << msg.info.height << " map @ "
           << msg.info.resolution << " m/cell";

    map = msg;
}

LOAD_MAP_STATUS loadMapFromYaml(const std::string& yaml_file,
                                commsgs::map_msgs::OccupancyGrid& map) {
    if (yaml_file.empty()) {
        AERROR << "[map_io] YAML file name is empty, can't load!";
        return MAP_DOES_NOT_EXIST;
    }

    AINFO << "[map_io] Loading yaml file:" << yaml_file;
    LoadParameters load_parameters;
    try {
        load_parameters = loadMapYaml(yaml_file);
    } catch (YAML::Exception& e) {
        AERROR << "[map_io] Failed processing YAML file "
               << " at position (" << e.mark.line << ":" << e.mark.column
               << ") for reason: " << e.what();
        return INVALID_MAP_METADATA;
    } catch (std::exception& e) {
        AERROR << "[map_io] Failed to parse map YAML loaded from file "
               << yaml_file << " for reason: " << e.what();
        return INVALID_MAP_METADATA;
    }
    try {
        loadMapFromFile(load_parameters, map);
    } catch (std::exception& e) {
        AERROR << "[map_io] Failed to load image file "
               << load_parameters.image_file_name
               << " for reason: " << e.what();
        return INVALID_MAP_DATA;
    }

    return LOAD_MAP_SUCCESS;
}

// === Map output part ===

/**
 * @brief Checks map saving parameters for consistency
 * @param save_parameters Map saving parameters.
 * NOTE: save_parameters could be updated during function execution.
 * @throw std::exception in case of inconsistent parameters
 */
void checkSaveParameters(SaveParameters& save_parameters) {
    // Checking map file name
    if (save_parameters.map_file_name == "") {
        // rclcpp::Clock clock(RCL_SYSTEM_TIME);
        // save_parameters.map_file_name = "map_" +
        // std::to_string(static_cast<int>(clock.now().seconds()));
        AWARN << "[map_io] Map file unspecified. Map will be saved to  "
              << save_parameters.map_file_name << " file";
    }

    // Checking thresholds
    if (save_parameters.occupied_thresh == 0.0) {
        save_parameters.occupied_thresh = 0.65;
        AWARN
            << "[map_io] Occupied threshold unspecified. Setting it to default "
               "value: "
            << save_parameters.occupied_thresh;
    }
    if (save_parameters.free_thresh == 0.0) {
        save_parameters.free_thresh = 0.25;
        AWARN << "[map_io] Free threshold unspecified. Setting it to default "
                 "value: "
              << save_parameters.free_thresh;
    }
    if (1.0 < save_parameters.occupied_thresh) {
        AERROR << "[map_io] Threshold_occupied must be 1.0 or less";
        throw std::runtime_error("Incorrect thresholds");
    }
    if (save_parameters.free_thresh < 0.0) {
        AERROR << "[map_io]  Free threshold must be 0.0 or greater";
        throw std::runtime_error("Incorrect thresholds");
    }
    if (save_parameters.occupied_thresh <= save_parameters.free_thresh) {
        AERROR << "[map_io] Threshold_free must be smaller than "
                  "threshold_occupied";
        throw std::runtime_error("Incorrect thresholds");
    }

    // Checking image format
    if (save_parameters.image_format == "") {
        save_parameters.image_format =
            save_parameters.mode == MapMode::Scale ? "png" : "pgm";
        AWARN << "[map_io] Image format unspecified. Setting it to: "
              << save_parameters.image_format;
    }

    std::transform(save_parameters.image_format.begin(),
                   save_parameters.image_format.end(),
                   save_parameters.image_format.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
    if (std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(),
                  save_parameters.image_format) == BLESSED_FORMATS.end()) {
        std::stringstream ss;
        bool first = true;
        for (auto& format_name : BLESSED_FORMATS) {
            if (!first) {
                ss << ", ";
            }
            ss << "'" << format_name << "'";
            first = false;
        }
        AWARN << "[map_io] Requested image format '"
              << save_parameters.image_format
              << "' is not one of the recommended formats: " << ss.str();
    }
    const std::string FALLBACK_FORMAT = "png";

    const std::vector<std::string> WRITABLE_FORMATS{"bmp", "pgm", "png", "jpg",
                                                    "jpeg"};
    if (std::find(WRITABLE_FORMATS.begin(), WRITABLE_FORMATS.end(),
                  save_parameters.image_format) == WRITABLE_FORMATS.end()) {
        AWARN << "[map_io] Format '" << save_parameters.image_format
              << "' may not be supported by OpenCV. Using '" << FALLBACK_FORMAT
              << "' instead";
        save_parameters.image_format = FALLBACK_FORMAT;
    }

    // Checking map mode
    if (save_parameters.mode == MapMode::Scale &&
        (save_parameters.image_format == "pgm" ||
         save_parameters.image_format == "jpg" ||
         save_parameters.image_format == "jpeg")) {
        AERROR
            << "[map_io] Map mode 'scale' requires transparency, but format '"
            << save_parameters.image_format
            << "' does not support it. Consider switching image format to "
               "'png'.";
    }
}

/**
 * @brief Tries to write map data into a file
 * @param map Occupancy grid data
 * @param save_parameters Map saving parameters
 * @throw std::expection in case of problem
 */
void tryWriteMapToFile(const commsgs::map_msgs::OccupancyGrid& map,
                       const SaveParameters& save_parameters) {
    AINFO << "[map_io] Received a " << map.info.width << " X "
          << map.info.height << " map @ " << map.info.resolution << " m/pix";

    std::string mapdatafile =
        save_parameters.map_file_name + "." + save_parameters.image_format;
    {
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        const int free_thresh_int =
            static_cast<int>(std::rint(save_parameters.free_thresh * 100.0));
        const int occupied_thresh_int =
            static_cast<int>(std::rint(save_parameters.occupied_thresh * 100.0));

        cv::Mat image;
        if (save_parameters.mode == MapMode::Scale) {
            image = cv::Mat(height, width, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        } else {
            image = cv::Mat(height, width, CV_8UC1, cv::Scalar(205));
        }

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                const int8_t map_cell = map.data[map.info.width *
                                                     (map.info.height - y - 1) +
                                                 x];

                switch (save_parameters.mode) {
                    case MapMode::Trinary: {
                        uint8_t gray = 205;
                        if (map_cell < 0 || map_cell > 100) {
                            gray = 205;
                        } else if (map_cell <= free_thresh_int) {
                            gray = 254;
                        } else if (occupied_thresh_int <= map_cell) {
                            gray = 0;
                        }
                        image.at<uint8_t>(y, x) = gray;
                        break;
                    }
                    case MapMode::Scale: {
                        cv::Vec4b& pixel = image.at<cv::Vec4b>(y, x);
                        if (map_cell < 0 || map_cell > 100) {
                            pixel = cv::Vec4b(128, 128, 128, 0);
                        } else {
                            const uint8_t gray = static_cast<uint8_t>(std::round(
                                (100.0 - map_cell) * 255.0 / 100.0));
                            pixel = cv::Vec4b(gray, gray, gray, 255);
                        }
                        break;
                    }
                    case MapMode::Raw: {
                        uint8_t gray = 255;
                        if (map_cell >= 0 && map_cell <= 100) {
                            gray = static_cast<uint8_t>(map_cell);
                        }
                        image.at<uint8_t>(y, x) = gray;
                        break;
                    }
                    default:
                        AERROR << "[map_io] Map mode should be Trinary, Scale "
                                  "or Raw";
                        throw std::runtime_error("Invalid map mode");
                }
            }
        }

        AINFO << "[map_io] Writing map occupancy data to " << mapdatafile;
        if (!cv::imwrite(mapdatafile, image)) {
            throw std::runtime_error("OpenCV failed to write map image: " +
                                   mapdatafile);
        }
    }

    std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
    {
        std::ofstream yaml(mapmetadatafile);

        commsgs::geometry_msgs::Quaternion orientation =
            map.info.origin.orientation;
        transform::tf2::Matrix3x3 mat(transform::tf2::Quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);

        const int file_name_index = mapdatafile.find_last_of("/\\");
        std::string image_name = mapdatafile.substr(file_name_index + 1);

        YAML::Emitter e;
        e << YAML::Precision(3);
        e << YAML::BeginMap;
        e << YAML::Key << "image" << YAML::Value << image_name;
        e << YAML::Key << "mode" << YAML::Value
          << map_mode_to_string(save_parameters.mode);
        e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
        e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq
          << map.info.origin.position.x << map.info.origin.position.y << yaw
          << YAML::EndSeq;
        e << YAML::Key << "negate" << YAML::Value << 0;
        e << YAML::Key << "occupied_thresh" << YAML::Value
          << save_parameters.occupied_thresh;
        e << YAML::Key << "free_thresh" << YAML::Value
          << save_parameters.free_thresh;

        if (!e.good()) {
            AERROR << "[map_io] YAML writer failed with an error "
                   << e.GetLastError() << ". The map metadata may be invalid.";
        }

        AINFO << "[map_io] Writing map metadata to " << mapmetadatafile;
        std::ofstream(mapmetadatafile) << e.c_str();
    }
    AINFO << "[map_io] Map saved";
}

bool saveMapToFile(const commsgs::map_msgs::OccupancyGrid& map,
                   const SaveParameters& save_parameters) {
    // Local copy of SaveParameters that might be modified by
    // checkSaveParameters()
    SaveParameters save_parameters_loc = save_parameters;

    try {
        // Checking map parameters for consistency
        checkSaveParameters(save_parameters_loc);
        tryWriteMapToFile(map, save_parameters_loc);
    } catch (std::exception& e) {
        AERROR << "[map_io] Failed to write map for reason: " << e.what();
        return false;
    }
    return true;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy