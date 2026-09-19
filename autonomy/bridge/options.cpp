/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

/**
 * @file options.cpp
 * @brief CLI11 parsing + CreateOptions / self-test helpers for bridge.
 */

#include "autonomy/bridge/options.hpp"

#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include <CLI/CLI.hpp>

#include "autolink/common/log.hpp"
#include "autonomy/bridge/tools/bootstrap.hpp"
#include "autonomy/bridge/tools/rpc_probe.hpp"
#include "autonomy/common/async_grpc/server.h"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/version.hpp"

namespace autonomy {
namespace bridge {
namespace {

bool UseAnsiColor() {
    if (const char* no = std::getenv("NO_COLOR");
        no != nullptr && no[0] != '\0') {
        return false;
    }
    if (const char* force = std::getenv("FORCE_COLOR");
        force != nullptr && force[0] != '\0' && force[0] != '0') {
        return true;
    }
    return isatty(STDOUT_FILENO) == 1;
}

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

struct HelpRow {
    std::string flags;
    std::string desc;
};

void AppendSection(std::ostringstream& out, const Ansi& c, const char* title,
                   const std::vector<HelpRow>& rows, std::size_t left_width) {
    out << '\n' << c.bold << c.cyan << title << c.reset << '\n';
    for (const HelpRow& row : rows) {
        std::ostringstream left;
        left << std::left << std::setw(static_cast<int>(left_width))
             << row.flags;
        out << "  " << c.green << left.str() << c.reset << "  " << row.desc
            << '\n';
    }
}

class ColorFormatter : public CLI::Formatter {
public:
    explicit ColorFormatter(bool color)
        : c_(color ? Ansi::Enabled() : Ansi::Disabled()) {
        enable_description_formatting(false);
        enable_footer_formatting(false);
    }

    std::string make_help(const CLI::App* /*app*/, std::string /*name*/,
                          CLI::AppFormatMode mode) const override {
        if (mode == CLI::AppFormatMode::Sub) {
            return {};
        }

        constexpr std::size_t kLeft = 30;
        std::ostringstream out;

        out << c_.bold << c_.cyan << "autonomy.bridge" << c_.reset << ' '
            << c_.green << "gRPC external API" << c_.reset << '\n';
        out << "  " << c_.dim
            << "Onboard adapter: automsgs.rpcs.* ↔ Autolink / autonomy.task"
            << c_.reset << '\n';

        out << '\n' << c_.bold << c_.cyan << "USAGE" << c_.reset << '\n';
        out << "  " << c_.yellow << "autonomy.bridge" << c_.reset
            << " [OPTIONS]              # start server\n";
        out << "  " << c_.yellow << "autonomy.bridge" << c_.reset
            << " call <METHOD> [OPTS]   # send any RPC\n";
        out << "  " << c_.yellow << "autonomy.bridge" << c_.reset
            << " list [SERVICE]         # list methods\n";
        out << "  " << c_.yellow << "autonomy.bridge" << c_.reset
            << " describe <SYMBOL>      # describe type\n";

        AppendSection(
            out, c_, "SERVER OPTIONS",
            {
                {"-h, --help", "Print help"},
                {"-V, --version", "Print version"},
                {"-c, --conf <FILE>",
                 "Protobuf text conf  [default: bridge.pb.txt]"},
                {"-n, --dry-run", "Load conf, print summary, exit"},
                {"-t, --self-test",
                 "Load conf + ApplyPlatform check, exit"},
                {"    --print-config", "Dump loaded BridgeOptions text"},
                {"    --host <HOST>", "Override grpc.host"},
                {"    --port <PORT>", "Override grpc.port"},
            },
            kLeft);

        AppendSection(
            out, c_, "CLIENT (call / list / describe)",
            {
                {"call <METHOD>", "Invoke RPC with JSON body"},
                {"  -d, --data <JSON|@file>", "Request JSON  [default: {}]"},
                {"  --target <HOST:PORT>",
                 "Server address  [default: 127.0.0.1:5005]"},
                {"  -H, --header <k:v>", "Metadata (repeatable)"},
                {"  --bearer <TOKEN>", "authorization: Bearer TOKEN"},
                {"  --robot-id <ID>", "x-robot-id metadata"},
                {"  --timeout <SEC>", "Deadline seconds  [default: 30]"},
                {"  --tls", "Use TLS instead of plaintext"},
                {"  -v, --verbose", "Print method path"},
                {"list [SERVICE]", "List services / methods"},
                {"describe <SYMBOL>", "Service, Method, or message type"},
            },
            kLeft);

        out << '\n' << c_.bold << c_.cyan << "EXAMPLES" << c_.reset << '\n';
        out << c_.dim
            << "  autonomy.bridge\n"
               "  autonomy.bridge -n\n"
               "  autonomy.bridge list\n"
               "  autonomy.bridge list SystemService\n"
               "  autonomy.bridge describe SystemService/Heartbeat\n"
               "  autonomy.bridge call SystemService/Heartbeat "
               "-d '{\"sequence\":1}'\n"
               "  autonomy.bridge call SystemService/GetInfo -d '{}'\n"
               "  autonomy.bridge call NavigationService/Navigate "
               "-d @nav.json --target 127.0.0.1:5005\n"
               "  autonomy.bridge call SystemService/Heartbeat "
               "--bearer secret --robot-id robot-1\n"
            << c_.reset;

        out << '\n' << c_.bold << c_.cyan << "ENVIRONMENT" << c_.reset << '\n';
        out << c_.dim
            << "  AUTONOMY_PATH     install / workspace prefix for conf lookup\n"
               "  NO_COLOR          disable ANSI colors\n"
               "  FORCE_COLOR       force ANSI colors\n"
            << c_.reset;

        out << '\n'
            << c_.dim
            << "  METHOD forms: SystemService/Heartbeat | "
               "automsgs.rpcs.system.SystemService/Heartbeat | unique name\n"
               "  Client-streaming / bidi: use grpcurl or "
               "automsgs/tools/cli/rpc-cli.py\n"
            << c_.reset;

        return out.str();
    }

private:
    Ansi c_;
};

std::string AuthModeName(proto::AuthMode mode) {
    switch (mode) {
        case proto::AUTH_MODE_TLS:
            return "TLS";
        case proto::AUTH_MODE_BEARER_TOKEN:
            return "BEARER_TOKEN";
        case proto::AUTH_MODE_NONE:
        default:
            return "NONE";
    }
}

}  // namespace

std::string VersionString() {
    const Ansi c = UseAnsiColor() ? Ansi::Enabled() : Ansi::Disabled();
    std::ostringstream oss;
    oss << c.bold << c.cyan << "autonomy.bridge" << c.reset << '\n'
        << c.dim << "  " << common::GetVersionInfo() << c.reset << '\n'
        << c.dim << "  " << common::GetBuildInfo() << c.reset << '\n'
        << c.dim << "  commit: " << common::GetGitCommitID() << c.reset;
    const std::string cuda = common::GetCudaInfo();
    if (!cuda.empty()) {
        oss << '\n' << c.dim << "  " << cuda << c.reset;
    }
    return oss.str();
}

ParseStatus ParseCommandLine(int argc, char** argv, CliOptions* out) {
    if (out == nullptr) {
        return ParseStatus::kExitError;
    }

    CliOptions opts;
    const bool color = UseAnsiColor();
    CLI::App app;
    app.name("autonomy.bridge");
    app.description("");
    app.formatter(std::make_shared<ColorFormatter>(color));
    app.set_help_flag("-h,--help", "Print help");
    app.set_version_flag("-V,--version", VersionString(), "Print version");
    app.require_subcommand(0, 1);

    app.add_option("-c,--conf", opts.conf_file, "Protobuf text conf")
        ->capture_default_str();
    app.add_flag("-n,--dry-run", opts.dry_run,
                 "Load conf and exit without listening");
    app.add_flag("-t,--self-test", opts.self_test,
                 "ApplyPlatform self-check then exit");
    app.add_flag("--print-config", opts.print_config,
                 "Dump loaded BridgeOptions");

    auto* host_opt =
        app.add_option("--host", opts.host, "Override grpc.host");
    auto* port_opt =
        app.add_option("--port", opts.port, "Override grpc.port");

    // call
    auto* call = app.add_subcommand("call", "Call any automsgs.rpcs method");
    call->add_option("method", opts.rpc_symbol, "Service/Method")
        ->required();
    call->add_option("-d,--data", opts.rpc_data, "JSON body or @file")
        ->capture_default_str();
    call->add_option("--target", opts.rpc_target, "host:port");
    call->add_option("-H,--header", opts.rpc_headers, "Metadata key:value");
    call->add_option("--bearer", opts.rpc_bearer, "Bearer token");
    call->add_option("--robot-id", opts.rpc_robot_id, "x-robot-id");
    call->add_option("--timeout", opts.rpc_timeout_sec, "Deadline seconds")
        ->capture_default_str();
    call->add_flag("--tls", opts.rpc_tls, "Use TLS");
    call->add_flag("-v,--verbose", opts.rpc_verbose, "Verbose");

    // list
    auto* list = app.add_subcommand("list", "List services / methods");
    list->add_option("service", opts.rpc_symbol, "Optional service filter");

    // describe
    auto* describe =
        app.add_subcommand("describe", "Describe service / method / message");
    describe->add_option("symbol", opts.rpc_symbol, "Symbol to describe")
        ->required();

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        const int code = app.exit(e);
        return (code == 0) ? ParseStatus::kExitOk : ParseStatus::kExitError;
    }

    opts.host_set = host_opt->count() > 0;
    opts.port_set = port_opt->count() > 0;
    if (*call) {
        opts.mode = CliMode::kCall;
    } else if (*list) {
        opts.mode = CliMode::kList;
    } else if (*describe) {
        opts.mode = CliMode::kDescribe;
    } else {
        opts.mode = CliMode::kServe;
    }

    *out = std::move(opts);
    return ParseStatus::kRun;
}

void ApplyCliOverrides(const CliOptions& cli, proto::BridgeOptions* options) {
    if (options == nullptr) {
        return;
    }
    if (cli.host_set) {
        options->mutable_grpc()->set_host(cli.host);
    }
    if (cli.port_set) {
        options->mutable_grpc()->set_port(cli.port);
    }
}

std::string ResolveRpcTarget(const CliOptions& cli) {
    if (!cli.rpc_target.empty()) {
        return cli.rpc_target;
    }
    const std::string host = cli.host_set && !cli.host.empty()
                                 ? cli.host
                                 : std::string("127.0.0.1");
    const std::uint32_t port = cli.port_set && cli.port != 0 ? cli.port : 5005u;
    return host + ":" + std::to_string(port);
}

int RunRpcClientMode(const CliOptions& cli) {
    tools::EnsureRpcDescriptorsLinked();
    if (cli.mode == CliMode::kList) {
        return tools::ListRpcMethods(cli.rpc_symbol).exit_code;
    }
    if (cli.mode == CliMode::kDescribe) {
        return tools::DescribeRpcSymbol(cli.rpc_symbol).exit_code;
    }
    if (cli.mode == CliMode::kCall) {
        tools::RpcCallOptions call;
        call.target = ResolveRpcTarget(cli);
        call.json_data = cli.rpc_data;
        call.headers = cli.rpc_headers;
        call.bearer_token = cli.rpc_bearer;
        call.robot_id = cli.rpc_robot_id;
        call.timeout_sec = cli.rpc_timeout_sec;
        call.tls = cli.rpc_tls;
        call.verbose = cli.rpc_verbose;
        return tools::CallRpc(cli.rpc_symbol, call).exit_code;
    }
    return EXIT_FAILURE;
}

proto::BridgeOptions CreateOptions(const std::string& conf_file) {
    proto::BridgeOptions options;
    const std::string file =
        conf_file.empty() ? std::string("bridge.pb.txt") : conf_file;
    CHECK(autonomy::common::LoadModuleConf("bridge", file, &options))
        << "Failed to load bridge conf: " << file;
    return options;
}

std::string SummarizeOptions(const proto::BridgeOptions& options) {
    const auto& g = options.grpc();
    const std::string host = g.host().empty() ? "127.0.0.1" : g.host();
    const std::uint32_t port = g.port() == 0 ? 5005u : g.port();
    std::ostringstream oss;
    oss << "listen=" << host << ':' << port
        << " grpc_threads=" << g.num_grpc_threads()
        << " event_threads=" << g.num_event_threads()
        << " worker_threads=" << g.num_worker_threads()
        << " health=" << (g.enable_health_check() ? "on" : "off")
        << " reflection=" << (g.enable_server_reflection() ? "on" : "off")
        << " metadata_interceptor="
        << (g.enable_metadata_interceptor() ? "on" : "off")
        << " auth=" << AuthModeName(g.auth_mode())
        << " ssl=" << (g.enable_ssl_encryption() ? "on" : "off")
        << " rate_limit=" << (g.enable_rate_limit() ? "on" : "off")
        << " otel=" << (g.enable_opentelemetry() ? "on" : "off");
    return oss.str();
}

bool RunPlatformSelfTest(const proto::BridgeOptions& options,
                         std::string* detail) {
    common::async_grpc::Server::Builder builder;
    const auto& g = options.grpc();
    const std::string host = g.host().empty() ? "127.0.0.1" : g.host();
    const std::uint32_t port = g.port() == 0 ? 5005u : g.port();
    builder.SetServerAddress(host + ":" + std::to_string(port));
    builder.SetNumGrpcThreads(g.num_grpc_threads() > 0 ? g.num_grpc_threads()
                                                       : 1);
    builder.SetNumEventThreads(g.num_event_threads() > 0 ? g.num_event_threads()
                                                         : 1);

    tools::PlatformApplyResult result;
    tools::ApplyPlatform(builder, g, &result);

    std::ostringstream oss;
    oss << "channel_args=" << (result.channel_args_applied ? "ok" : "fail")
        << " credentials=" << (result.credentials_applied ? "ok" : "fail")
        << " health=" << (result.health_enabled ? "on" : "off")
        << " reflection=" << (result.reflection_enabled ? "on" : "off")
        << " reflection_lib="
        << (result.reflection_library_available ? "yes" : "no")
        << " interceptors={logging=" << result.interceptors.logging
        << " auth=" << result.interceptors.auth
        << " metadata=" << result.interceptors.metadata
        << " rate_limit=" << result.interceptors.rate_limit
        << " otel=" << result.interceptors.otel << "}";
    if (detail != nullptr) {
        *detail = oss.str();
    }

    return result.channel_args_applied && result.credentials_applied;
}

}  // namespace bridge
}  // namespace autonomy
