/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/transport/credentials_factory.hpp"

#include <fstream>

#include "autolink/common/log.hpp"
#include "autonomy/bridge/policy/auth/tls_options.hpp"

namespace autonomy {
namespace bridge {
namespace tools {
namespace {

bool FileReadable(const std::string& path) {
    std::ifstream in(path);
    return static_cast<bool>(in);
}

}  // namespace

std::shared_ptr<::grpc::ServerCredentials> CredentialsFactory::Create(
    const proto::GrpcOptions& options) {
    const auto tls = policy::TlsOptions::FromGrpcOptions(options);
    if (!tls.enabled) {
        return ::grpc::InsecureServerCredentials();
    }
    if (tls.cert_path.empty() || tls.key_path.empty() ||
        !FileReadable(tls.cert_path) || !FileReadable(tls.key_path)) {
        AERROR << "CredentialsFactory: TLS enabled but cert/key missing or "
                  "unreadable; falling back to insecure credentials";
        return ::grpc::InsecureServerCredentials();
    }
    ::grpc::SslServerCredentialsOptions ssl_opts(
        tls.require_client_cert
            ? GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY
            : GRPC_SSL_DONT_REQUEST_CLIENT_CERTIFICATE);
    ::grpc::SslServerCredentialsOptions::PemKeyCertPair pair;
    {
        std::ifstream cert(tls.cert_path);
        std::ifstream key(tls.key_path);
        pair.cert_chain.assign((std::istreambuf_iterator<char>(cert)),
                             std::istreambuf_iterator<char>());
        pair.private_key.assign((std::istreambuf_iterator<char>(key)),
                                std::istreambuf_iterator<char>());
    }
    ssl_opts.pem_key_cert_pairs.push_back(pair);
    if (!tls.ca_path.empty() && FileReadable(tls.ca_path)) {
        std::ifstream ca(tls.ca_path);
        ssl_opts.pem_root_certs.assign((std::istreambuf_iterator<char>(ca)),
                                      std::istreambuf_iterator<char>());
    }
    return ::grpc::SslServerCredentials(ssl_opts);
}

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
