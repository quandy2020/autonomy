/*
 * Copyright 2017 The Cartographer Authors
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

#include <map>
#include <string>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autonomy/common/cli_options.hpp"
#include "autonomy/localization/cartographer/io/file_writer.hpp"
#include "autonomy/localization/cartographer/io/proto_stream.hpp"
#include "autonomy/localization/cartographer/io/proto_stream_deserializer.hpp"
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/mapping/value_conversion_tables.hpp"
#include "autonomy/localization/cartographer/node/map_io.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {
namespace {

void Run(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution) {
    ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
    ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

    LOG(INFO) << "Loading submap slices from serialized data.";
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;
    ::cartographer::mapping::ValueConversionTables conversion_tables;
    ::cartographer::io::DeserializeAndFillSubmapSlices(&deserializer,
                                                       &submap_slices,
                                                       &conversion_tables);
    CHECK(reader.eof());

    LOG(INFO) << "Generating combined map image from submap slices.";
    auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

    ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");
    ::cartographer::io::Image image(std::move(result.surface));
    WritePgm(image, resolution, &pgm_writer);

    const Eigen::Vector2d origin(
        -result.origin().x() * resolution,
        (result.origin().y() - image.height()) * resolution);

    ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
    WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
}

}  // namespace
}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy

int main(int argc, char** argv) {
    std::string pbstream_filename;
    std::string map_filestem = "map";
    double resolution = 0.05;

    CLI::App app{"Draw a map (PGM/YAML) from a Cartographer pbstream."};
    app.add_option("--pbstream_filename", pbstream_filename,
                   "Filename of a pbstream to draw a map from.")
        ->required();
    app.add_option("--map_filestem", map_filestem, "Stem of the output files.")
        ->capture_default_str();
    app.add_option("--resolution", resolution, "Resolution of a grid cell in the drawn map.")
        ->capture_default_str();

    autonomy::common::ParseOrExit(app, argc, argv);
    google::InitGoogleLogging(argv[0]);

    CHECK(!map_filestem.empty()) << "--map_filestem is missing.";

    autonomy::localization::cartographer::node::Run(pbstream_filename, map_filestem, resolution);
    return EXIT_SUCCESS;
}
