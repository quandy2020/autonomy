/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/rpc_probe.hpp"

#include <cstdlib>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/json_util.h>
#include <grpcpp/generic/generic_stub.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/rpc_method.h>
#include <grpcpp/support/byte_buffer.h>
#include <grpcpp/support/sync_stream.h>

#include <automsgs/rpcs/charge.pb.h>
#include <automsgs/rpcs/exploration.pb.h>
#include <automsgs/rpcs/follow.pb.h>
#include <automsgs/rpcs/localization.pb.h>
#include <automsgs/rpcs/mapping.pb.h>
#include <automsgs/rpcs/navigation.pb.h>
#include <automsgs/rpcs/sensor.pb.h>
#include <automsgs/rpcs/system.pb.h>
#include <automsgs/rpcs/teleop.pb.h>
#include <automsgs/rpcs/voice.pb.h>

namespace autonomy {
namespace bridge {
namespace tools {
namespace {

constexpr const char* kKnownServices[] = {
    "automsgs.rpcs.system.SystemService",
    "automsgs.rpcs.navigation.NavigationService",
    "automsgs.rpcs.follow.FollowService",
    "automsgs.rpcs.teleop.TeleopService",
    "automsgs.rpcs.charge.ChargeService",
    "automsgs.rpcs.mapping.MapService",
    "automsgs.rpcs.exploration.ExplorationService",
    "automsgs.rpcs.voice.VoiceService",
    "automsgs.rpcs.localization.LocalizationService",
    "automsgs.rpcs.sensor.SensorService",
};

void TouchDescriptors() {
    // Force-link .pb.cc so ServiceDescriptors land in generated_pool.
    (void)::automsgs::rpcs::system::HeartbeatRequest::descriptor();
    (void)::automsgs::rpcs::navigation::NavigateRequest::descriptor();
    (void)::automsgs::rpcs::follow::FollowRequest::descriptor();
    (void)::automsgs::rpcs::teleop::DriveOnHeadingRequest::descriptor();
    (void)::automsgs::rpcs::charge::ReturnRequest::descriptor();
    (void)::automsgs::rpcs::mapping::ListMapsRequest::descriptor();
    (void)::automsgs::rpcs::exploration::ExploreRequest::descriptor();
    (void)::automsgs::rpcs::voice::VoiceCommandRequest::descriptor();
    (void)::automsgs::rpcs::localization::GetPoseRequest::descriptor();
    (void)::automsgs::rpcs::sensor::ListSensorsRequest::descriptor();
}

const google::protobuf::ServiceDescriptor* FindService(
    const std::string& name) {
    const auto* pool = google::protobuf::DescriptorPool::generated_pool();
    if (const auto* svc = pool->FindServiceByName(name)) {
        return svc;
    }
    for (const char* full : kKnownServices) {
        const auto* svc = pool->FindServiceByName(full);
        if (svc == nullptr) {
            continue;
        }
        if (svc->name() == name || std::string(full) == name) {
            return svc;
        }
        const std::string full_s(full);
        if (full_s.size() > name.size() &&
            full_s.compare(full_s.size() - name.size(), name.size(), name) ==
                0 &&
            full_s[full_s.size() - name.size() - 1] == '.') {
            return svc;
        }
    }
    return nullptr;
}

std::string LoadDataMaybeFile(const std::string& data) {
    if (data.empty()) {
        return "{}";
    }
    if (data[0] != '@') {
        return data;
    }
    std::ifstream in(data.substr(1));
    if (!in) {
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool MessageToJson(const google::protobuf::Message& msg, std::string* out) {
    google::protobuf::util::JsonPrintOptions opts;
    opts.add_whitespace = true;
    opts.preserve_proto_field_names = true;
    const auto status =
        google::protobuf::util::MessageToJsonString(msg, out, opts);
    return status.ok();
}

bool JsonToMessage(const std::string& json, google::protobuf::Message* msg,
                   std::string* error) {
    google::protobuf::util::JsonParseOptions opts;
    opts.ignore_unknown_fields = true;
    const auto status =
        google::protobuf::util::JsonStringToMessage(json, msg, opts);
    if (!status.ok()) {
        if (error) {
            *error = std::string(status.message());
        }
        return false;
    }
    return true;
}

bool SerializeToByteBuffer(const google::protobuf::Message& msg,
                           ::grpc::ByteBuffer* out) {
    std::string bytes;
    if (!msg.SerializeToString(&bytes)) {
        return false;
    }
    ::grpc::Slice slice(bytes);
    ::grpc::ByteBuffer tmp(&slice, 1);
    out->Swap(&tmp);
    return true;
}

bool ParseFromByteBuffer(const ::grpc::ByteBuffer& buffer,
                         google::protobuf::Message* msg) {
    std::vector<::grpc::Slice> slices;
    if (!buffer.Dump(&slices).ok()) {
        return false;
    }
    std::string bytes;
    for (const auto& s : slices) {
        bytes.append(reinterpret_cast<const char*>(s.begin()), s.size());
    }
    return msg->ParseFromString(bytes);
}

bool CqNext(::grpc::CompletionQueue* cq, void* expected) {
    void* tag = nullptr;
    bool ok = false;
    if (!cq->Next(&tag, &ok) || !ok || tag != expected) {
        return false;
    }
    return true;
}

std::string MethodPath(const google::protobuf::MethodDescriptor* method) {
    return "/" + method->service()->full_name() + "/" + method->name();
}

std::string ClientStreamingLabel(
    const google::protobuf::MethodDescriptor* method) {
    if (method->client_streaming() && method->server_streaming()) {
        return "bidi-streaming";
    }
    if (method->client_streaming()) {
        return "client-streaming";
    }
    if (method->server_streaming()) {
        return "server-streaming";
    }
    return "unary";
}

void ApplyMetadata(::grpc::ClientContext* ctx, const RpcCallOptions& options) {
    for (const auto& h : options.headers) {
        const auto pos = h.find(':');
        if (pos == std::string::npos || pos == 0) {
            continue;
        }
        std::string key = h.substr(0, pos);
        std::string value = h.substr(pos + 1);
        while (!value.empty() && value[0] == ' ') {
            value.erase(value.begin());
        }
        ctx->AddMetadata(key, value);
    }
    if (!options.bearer_token.empty()) {
        ctx->AddMetadata("authorization", "Bearer " + options.bearer_token);
    }
    if (!options.robot_id.empty()) {
        ctx->AddMetadata("x-robot-id", options.robot_id);
    }
    if (options.timeout_sec > 0) {
        ctx->set_deadline(std::chrono::system_clock::now() +
                          std::chrono::seconds(options.timeout_sec));
    }
}

RpcProbeResult Fail(const std::string& msg, int code = 1) {
    RpcProbeResult r;
    r.ok = false;
    r.exit_code = code;
    r.message = msg;
    std::cerr << msg << '\n';
    return r;
}

RpcProbeResult Ok(const std::string& msg = {}) {
    RpcProbeResult r;
    r.ok = true;
    r.exit_code = 0;
    r.message = msg;
    return r;
}

}  // namespace

void EnsureRpcDescriptorsLinked() { TouchDescriptors(); }

bool ResolveRpcMethod(const std::string& raw, std::string* full_path,
                      std::string* error) {
    EnsureRpcDescriptorsLinked();
    if (full_path == nullptr) {
        if (error) {
            *error = "null full_path";
        }
        return false;
    }

    std::string input = raw;
    while (!input.empty() && input[0] == '/') {
        input.erase(input.begin());
    }

    // Full package.Service/Method
    if (input.find('/') != std::string::npos) {
        const auto slash = input.find('/');
        const std::string svc_part = input.substr(0, slash);
        const std::string method_part = input.substr(slash + 1);
        const auto* svc = FindService(svc_part);
        if (svc == nullptr) {
            if (error) {
                *error = "unknown service: " + svc_part;
            }
            return false;
        }
        const auto* method = svc->FindMethodByName(method_part);
        if (method == nullptr) {
            if (error) {
                *error = "unknown method: " + input;
            }
            return false;
        }
        *full_path = MethodPath(method);
        return true;
    }

    // Unique method name
    std::vector<const google::protobuf::MethodDescriptor*> matches;
    const auto* pool = google::protobuf::DescriptorPool::generated_pool();
    for (const char* full : kKnownServices) {
        const auto* svc = pool->FindServiceByName(full);
        if (svc == nullptr) {
            continue;
        }
        const auto* method = svc->FindMethodByName(input);
        if (method != nullptr) {
            matches.push_back(method);
        }
    }
    if (matches.size() == 1) {
        *full_path = MethodPath(matches[0]);
        return true;
    }
    if (matches.empty()) {
        if (error) {
            *error = "unknown method: " + raw + " (try: autonomy.bridge list)";
        }
        return false;
    }
    std::ostringstream oss;
    oss << "ambiguous method '" << raw << "', matches:";
    for (const auto* m : matches) {
        oss << "\n  " << MethodPath(m);
    }
    if (error) {
        *error = oss.str();
    }
    return false;
}

RpcProbeResult ListRpcMethods(const std::string& service_filter) {
    EnsureRpcDescriptorsLinked();
    const auto* pool = google::protobuf::DescriptorPool::generated_pool();
    int services = 0;
    int methods = 0;
    for (const char* full : kKnownServices) {
        const auto* svc = pool->FindServiceByName(full);
        if (svc == nullptr) {
            continue;
        }
        if (!service_filter.empty() && svc->name() != service_filter &&
            svc->full_name() != service_filter) {
            continue;
        }
        ++services;
        std::cout << svc->full_name() << '\n';
        for (int i = 0; i < svc->method_count(); ++i) {
            const auto* m = svc->method(i);
            std::cout << "  " << MethodPath(m) << "  ["
                      << ClientStreamingLabel(m) << "]\n";
            ++methods;
        }
    }
    if (services == 0) {
        return Fail(service_filter.empty()
                        ? "no services found (descriptors not linked?)"
                        : "unknown service: " + service_filter);
    }
    std::cout << "\n# " << services << " services, " << methods << " methods\n";
    return Ok();
}

RpcProbeResult DescribeRpcSymbol(const std::string& symbol) {
    EnsureRpcDescriptorsLinked();
    const auto* pool = google::protobuf::DescriptorPool::generated_pool();

    if (const auto* svc = FindService(symbol)) {
        std::cout << "service " << svc->full_name() << " {\n";
        for (int i = 0; i < svc->method_count(); ++i) {
            const auto* m = svc->method(i);
            std::cout << "  rpc " << m->name() << " ("
                      << m->input_type()->full_name() << ") returns ("
                      << m->output_type()->full_name() << ");  // "
                      << ClientStreamingLabel(m) << "\n";
        }
        std::cout << "}\n";
        return Ok();
    }

    std::string path;
    std::string err;
    if (ResolveRpcMethod(symbol, &path, &err)) {
        // path is /pkg.Svc/Method
        const auto slash = path.find_last_of('/');
        const auto svc_name = path.substr(1, slash - 1);
        const auto method_name = path.substr(slash + 1);
        const auto* svc = pool->FindServiceByName(svc_name);
        const auto* m = svc ? svc->FindMethodByName(method_name) : nullptr;
        if (m != nullptr) {
            std::cout << "rpc " << m->full_name() << "\n"
                      << "  type:     " << ClientStreamingLabel(m) << "\n"
                      << "  request:  " << m->input_type()->full_name() << "\n"
                      << "  response: " << m->output_type()->full_name() << "\n"
                      << "  path:     " << path << "\n";
            std::cout << "\n// request fields\n";
            google::protobuf::TextFormat::Printer printer;
            // Print descriptor debug
            for (int i = 0; i < m->input_type()->field_count(); ++i) {
                const auto* f = m->input_type()->field(i);
                std::cout << "  " << f->type_name() << " " << f->name() << " = "
                          << f->number() << "\n";
            }
            return Ok();
        }
    }

    const auto* msg = pool->FindMessageTypeByName(symbol);
    if (msg == nullptr) {
        // try suffix match
        for (const char* full : kKnownServices) {
            (void)full;
        }
        return Fail("unknown symbol: " + symbol + "\n" + err);
    }
    std::cout << "message " << msg->full_name() << " {\n";
    for (int i = 0; i < msg->field_count(); ++i) {
        const auto* f = msg->field(i);
        std::cout << "  " << f->type_name() << " " << f->name() << " = "
                  << f->number() << "\n";
    }
    std::cout << "}\n";
    return Ok();
}

RpcProbeResult CallRpc(const std::string& method_raw,
                       const RpcCallOptions& options) {
    EnsureRpcDescriptorsLinked();

    std::string path;
    std::string err;
    if (!ResolveRpcMethod(method_raw, &path, &err)) {
        return Fail(err);
    }

    const auto slash = path.find_last_of('/');
    const auto svc_name = path.substr(1, slash - 1);
    const auto method_name = path.substr(slash + 1);
    const auto* pool = google::protobuf::DescriptorPool::generated_pool();
    const auto* svc = pool->FindServiceByName(svc_name);
    const auto* method = svc ? svc->FindMethodByName(method_name) : nullptr;
    if (method == nullptr) {
        return Fail("internal: resolved path not found: " + path);
    }

    if (method->client_streaming()) {
        return Fail(std::string("client/bidi streaming not supported in "
                                "autonomy.bridge call (") +
                    ClientStreamingLabel(method) +
                    "). Use grpcurl or rpc-cli.py for " + path);
    }

    const std::string json = LoadDataMaybeFile(options.json_data);
    if (json.empty() && !options.json_data.empty() &&
        options.json_data[0] == '@') {
        return Fail("cannot read data file: " + options.json_data);
    }

    google::protobuf::DynamicMessageFactory factory;
    std::unique_ptr<google::protobuf::Message> request(
        factory.GetPrototype(method->input_type())->New());
    std::unique_ptr<google::protobuf::Message> response(
        factory.GetPrototype(method->output_type())->New());

    if (!JsonToMessage(json, request.get(), &err)) {
        return Fail("invalid JSON for " + method->input_type()->full_name() +
                    ": " + err);
    }

    ::grpc::ByteBuffer request_buf;
    if (!SerializeToByteBuffer(*request, &request_buf)) {
        return Fail("failed to serialize request");
    }

    std::shared_ptr<::grpc::Channel> channel;
    if (options.tls) {
        channel = ::grpc::CreateChannel(options.target,
                                        ::grpc::SslCredentials(
                                            ::grpc::SslCredentialsOptions()));
    } else {
        channel = ::grpc::CreateChannel(
            options.target, ::grpc::InsecureChannelCredentials());
    }

    if (options.verbose) {
        std::cerr << "+ " << path << " @ " << options.target << " ["
                  << ClientStreamingLabel(method) << "]\n";
    }

    ::grpc::ClientContext ctx;
    ApplyMetadata(&ctx, options);

    if (!method->server_streaming()) {
        // Unary
        ::grpc::GenericStub stub(channel);
        ::grpc::CompletionQueue cq;
        void* tag = reinterpret_cast<void*>(1);
        auto reader =
            stub.PrepareUnaryCall(&ctx, path, request_buf, &cq);
        if (!reader) {
            return Fail("PrepareUnaryCall failed");
        }
        reader->StartCall();
        ::grpc::ByteBuffer response_buf;
        ::grpc::Status status;
        reader->Finish(&response_buf, &status, tag);
        if (!CqNext(&cq, tag)) {
            return Fail("unary RPC completion queue failed");
        }
        if (!status.ok()) {
            return Fail("RPC failed: " + status.error_message() +
                        " (code=" +
                        std::to_string(static_cast<int>(status.error_code())) +
                        ")");
        }
        if (!ParseFromByteBuffer(response_buf, response.get())) {
            return Fail("failed to parse response");
        }
        std::string out;
        if (!MessageToJson(*response, &out)) {
            return Fail("failed to print response JSON");
        }
        std::cout << out << std::flush;
        if (!out.empty() && out.back() != '\n') {
            std::cout << '\n';
        }
        return Ok();
    }

    // Server-streaming (sync ClientReader)
    const ::grpc::internal::RpcMethod rpc(
        path.c_str(), /*suffix_for_stats=*/nullptr,
        ::grpc::internal::RpcMethod::SERVER_STREAMING);
    std::unique_ptr<::grpc::ClientReader<::grpc::ByteBuffer>> reader(
        ::grpc::internal::ClientReaderFactory<::grpc::ByteBuffer>::Create(
            channel.get(), rpc, &ctx, request_buf));

    int frames = 0;
    ::grpc::ByteBuffer response_buf;
    while (reader->Read(&response_buf)) {
        if (!ParseFromByteBuffer(response_buf, response.get())) {
            return Fail("failed to parse stream frame");
        }
        std::string out;
        if (!MessageToJson(*response, &out)) {
            return Fail("failed to print stream frame JSON");
        }
        std::cout << out;
        if (!out.empty() && out.back() != '\n') {
            std::cout << '\n';
        }
        std::cout << std::flush;
        ++frames;
        response->Clear();
        response_buf.Clear();
    }
    const ::grpc::Status status = reader->Finish();
    if (!status.ok()) {
        return Fail("stream RPC failed after " + std::to_string(frames) +
                    " frames: " + status.error_message());
    }
    if (options.verbose) {
        std::cerr << "# received " << frames << " frames\n";
    }
    return Ok();
}

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
