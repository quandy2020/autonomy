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

#include <cstdlib>
#include <string>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autonomy/common/cli_options.hpp"
#include "autonomy/localization/cartographer/io/internal/pbstream_info.hpp"
#include "autonomy/localization/cartographer/io/internal/pbstream_migrate.hpp"

int main(int argc, char** argv) {
    CLI::App app{"Swiss Army knife for pbstreams."};
    app.require_subcommand(1, 1);

    cartographer::io::PbstreamInfoOptions info_options;
    auto* info_cmd = app.add_subcommand("info", "Prints summary of pbstream.");
    info_cmd->add_option("pbstream_filename", info_options.pbstream_filename,
                         "Pbstream file to summarize.")
        ->required();
    info_cmd->add_flag("--all_debug_strings", info_options.all_debug_strings,
                       "Print debug strings of all serialized data.");

    cartographer::io::PbstreamMigrateOptions migrate_options;
    auto* migrate_cmd =
        app.add_subcommand("migrate", "Migrates pbstream to the new submap format.");
    migrate_cmd->add_option("input_filename", migrate_options.input_filename, "Input pbstream.")
        ->required();
    migrate_cmd->add_option("output_filename", migrate_options.output_filename, "Output pbstream.")
        ->required();
    migrate_cmd->add_option("--include_unfinished_submaps", migrate_options.include_unfinished_submaps,
                            "Whether to include unfinished submaps in the output.")
        ->default_val(true);

    autonomy::common::ParseOrExit(app, argc, argv);
    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

    if (*info_cmd) {
        return ::cartographer::io::pbstream_info(info_options);
    }
    if (*migrate_cmd) {
        return ::cartographer::io::pbstream_migrate(migrate_options);
    }

    return EXIT_FAILURE;
}
