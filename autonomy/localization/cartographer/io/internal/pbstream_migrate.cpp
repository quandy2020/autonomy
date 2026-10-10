/*
 * Copyright 2018 The Cartographer Authors
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

#include "autonomy/localization/cartographer/io/internal/pbstream_migrate.hpp"

#include "autonomy/localization/cartographer/io/proto_stream.hpp"
#include "autonomy/localization/cartographer/io/serialization_format_migration.hpp"
#include "glog/logging.h"

namespace cartographer {
namespace io {

int pbstream_migrate(const PbstreamMigrateOptions& options) {
    if (options.input_filename.empty() || options.output_filename.empty()) {
        LOG(ERROR) << "pbstream migrate requires <input_filename> <output_filename>.";
        return EXIT_FAILURE;
    }

    cartographer::io::ProtoStreamReader input(options.input_filename);
    cartographer::io::ProtoStreamWriter output(options.output_filename);
    LOG(INFO) << "Migrating serialization format 1 in \"" << options.input_filename
              << "\" to serialization format 2 in \"" << options.output_filename << "\"";
    cartographer::io::MigrateStreamVersion1ToVersion2(&input, &output, options.include_unfinished_submaps);
    CHECK(output.Close()) << "Could not write migrated pbstream file to: " << options.output_filename;

    return EXIT_SUCCESS;
}

}  // namespace io
}  // namespace cartographer
