/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file options.cpp
 * @brief CLI parsing and colored --help / --version for autodriver.
 */

#include "options.hpp"

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include <CLI/CLI.hpp>

#include "autodriver/conf/conf.hpp"
#include "autodriver/config_loader.hpp"

namespace autodriver {
namespace {

/**
 * @brief True when ANSI colors should be used on stdout.
 */
bool UseAnsiColor() {
    if (const char* no = std::getenv("NO_COLOR"); no != nullptr && no[0] != '\0') {
        return false;
    }
    if (const char* force = std::getenv("FORCE_COLOR");
        force != nullptr && force[0] != '\0' && force[0] != '0') {
        return true;
    }
    if (const char* force = std::getenv("CLICOLOR_FORCE");
        force != nullptr && force[0] != '\0' && force[0] != '0') {
        return true;
    }
    return isatty(STDOUT_FILENO) == 1;
}

/**
 * @brief ANSI escape fragments (empty strings when color is disabled).
 */
struct Ansi {
    const char* reset;
    const char* bold;
    const char* dim;
    const char* cyan;
    const char* green;
    const char* yellow;

    static Ansi Enabled() {
        return {"\033[0m", "\033[1m", "\033[2m", "\033[36m", "\033[32m",
                "\033[33m"};
    }
    static Ansi Disabled() { return {"", "", "", "", "", ""}; }
};

/**
 * @brief One left/right row in a help section.
 */
struct HelpRow {
    /** @brief Left column flags text (no ANSI). */
    std::string flags;
    /** @brief Right column description. */
    std::string desc;
};

/**
 * @brief Appends a titled two-column section to @p out.
 */
void AppendSection(std::ostringstream& out, const Ansi& c, const char* title,
                   const std::vector<HelpRow>& rows, std::size_t left_width) {
    out << '\n' << c.bold << c.cyan << title << c.reset << '\n';
    for (const HelpRow& row : rows) {
        std::ostringstream left;
        left << std::left << std::setw(static_cast<int>(left_width)) << row.flags;
        out << "  " << c.green << left.str() << c.reset << "  " << row.desc
            << '\n';
    }
}

/**
 * @brief Hand-laid help (cargo / ripgrep style). CLI11 only parses.
 */
class ColorFormatter : public CLI::Formatter {
public:
    /**
     * @brief Construct a formatter with optional ANSI coloring.
     * @param[in] color When true, emit ANSI color codes.
     */
    explicit ColorFormatter(bool color)
        : c_(color ? Ansi::Enabled() : Ansi::Disabled()) {
        enable_description_formatting(false);
        enable_footer_formatting(false);
    }

    /**
     * @brief Render the top-level autodriver help text.
     */
    std::string make_help(const CLI::App* /*app*/, std::string /*name*/,
                          CLI::AppFormatMode mode) const override {
        if (mode == CLI::AppFormatMode::Sub) {
            return {};
        }

        constexpr std::size_t kLeft = 28;
        std::ostringstream out;

        out << c_.bold << c_.cyan << conf::kPackageName << c_.reset << ' '
            << c_.green << conf::kFullVersion << c_.reset << '\n';
        if (conf::kPackageDescription != nullptr &&
            conf::kPackageDescription[0] != '\0') {
            out << "  " << c_.dim << conf::kPackageDescription << c_.reset
                << '\n';
        }
        out << "  " << c_.dim << conf::kGitBranch << " @ " << conf::kGitCommit
            << " · " << conf::kGitAuthor << " <" << conf::kGitEmail << "> · "
            << conf::kGitCommitDate << c_.reset << '\n';

        out << '\n' << c_.bold << c_.cyan << "USAGE" << c_.reset << '\n';
        out << "  " << c_.yellow << "autodriver" << c_.reset
            << " [OPTIONS] [CONFIG_DIR] [CONFIG_FILE]\n";

        AppendSection(out, c_, "OPTIONS",
                      {
                          {"-h, --help", "Print help"},
                          {"-V, --version", "Print version"},
                          {"-n, --dry-run",
                           "Load config and exit (do not start hardware)"},
                          {"    --no-udev", "Disable udev hotplug"},
                      },
                      kLeft);

        AppendSection(
            out, c_, "CONFIG",
            {
                {"-c, --config-dir <DIR>",
                 "Config root (contains config/)  [env: AUTODRIVER_PATH]"},
                {"    --config-file <FILE>",
                 std::string("Config basename  [default: ") +
                     kDefaultConfigBasename + "]"},
                {"CONFIG_DIR", "Same as --config-dir (positional)"},
                {"CONFIG_FILE", "Same as --config-file (positional)"},
            },
            kLeft);

        out << '\n' << c_.bold << c_.cyan << "EXAMPLES" << c_.reset << '\n';
        out << c_.dim
            << "  autodriver\n"
               "  autodriver /path/to/autodriver\n"
               "  autodriver -c /path/to/autodriver "
               "--config-file autodriver_hardware.yaml\n"
               "  autodriver -n\n"
               "  autodriver --no-udev\n"
            << c_.reset;

        out << '\n' << c_.bold << c_.cyan << "ENVIRONMENT" << c_.reset << '\n';
        out << c_.dim
            << "  AUTODRIVER_PATH                 package root (must contain config/)\n"
               "  AUTODRIVER_DISTRIBUTION_HOME    install-tree fallback\n"
               "  AUTODRIVER_PLUGIN_DIR           default plugin .so path\n"
            << c_.reset;

        out << '\n'
            << c_.dim
            << "  Runtime knobs (node_name, plugin_dir, pose_channel, …) "
               "belong in YAML.\n"
            << c_.reset;

        return out.str();
    }

private:
    Ansi c_;
};

}  // namespace

std::string VersionString() {
    const Ansi c = UseAnsiColor() ? Ansi::Enabled() : Ansi::Disabled();
    std::ostringstream oss;
    oss << c.bold << c.cyan << conf::kPackageName << c.reset << ' '
        << c.green << conf::kFullVersion << c.reset;
    if (conf::kPackageDescription != nullptr &&
        conf::kPackageDescription[0] != '\0') {
        oss << c.dim << " — " << conf::kPackageDescription << c.reset;
    }
    oss << '\n'
        << c.dim << "  version:   " << conf::kVersion << c.reset << '\n'
        << c.dim << "  branch:    " << conf::kGitBranch << c.reset << '\n'
        << c.dim << "  commit:    " << conf::kGitCommit;
    if (conf::kGitDirty != nullptr &&
        std::strcmp(conf::kGitDirty, "true") == 0) {
        oss << " (dirty)";
    }
    oss << c.reset << '\n'
        << c.dim << "  describe:  " << conf::kGitDescribe << c.reset << '\n'
        << c.dim << "  author:    " << conf::kGitAuthor << " <"
        << conf::kGitEmail << ">" << c.reset << '\n'
        << c.dim << "  committed: " << conf::kGitCommitDate << c.reset << '\n'
        << c.dim << "  built:     " << conf::kBuildTime << c.reset;
    return oss.str();
}

ParseStatus ParseCommandLine(int argc, char** argv, Options* out) {
    if (out == nullptr) {
        return ParseStatus::kExitError;
    }

    Options opts;
    opts.config_file = kDefaultConfigBasename;

    const bool color = UseAnsiColor();
    CLI::App app;
    app.name("autodriver");
    app.description("");
    app.formatter(std::make_shared<ColorFormatter>(color));
    app.set_help_flag("-h,--help", "Print help");
    app.set_version_flag("-V,--version", VersionString(), "Print version");

    // Google CLI: -n = dry-run; -f reserved for --force (not used here).
    // Config basename is long-only: --config-file (kebab-case).
    app.add_option("config_directory,-c,--config-dir", opts.config_directory,
                   "Config root")
        ->envname("AUTODRIVER_PATH");
    app.add_option("config_file,--config-file", opts.config_file, "Config file")
        ->capture_default_str();
    app.add_flag("-n,--dry-run", opts.dry_run, "Dry run");
    app.add_flag("--no-udev", opts.disable_udev, "Disable udev");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        const int code = app.exit(e);
        return (code == 0) ? ParseStatus::kExitOk : ParseStatus::kExitError;
    }

    *out = std::move(opts);
    return ParseStatus::kRun;
}

}  // namespace autodriver
