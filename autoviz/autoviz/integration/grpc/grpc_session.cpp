/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/grpc/grpc_session.hpp"

#if AUTOVIZ_ENABLE_GRPC
#include <grpcpp/security/credentials.h>
#include <grpcpp/security/tls_credentials_options.h>
#include <grpcpp/support/channel_arguments.h>
#endif

namespace autoviz {
namespace integration {
namespace grpc_client {
namespace {

void SetError(std::string* err, const std::string& message) {
  if (err != nullptr) {
    *err = message;
  }
}

bool StartsWith(const std::string& s, const char* prefix) {
  const size_t n = std::char_traits<char>::length(prefix);
  return s.size() >= n && s.compare(0, n, prefix) == 0;
}

}  // namespace

std::string NormalizeGrpcTarget(const std::string& target) {
  std::string out = target;
  while (!out.empty() && (out.front() == ' ' || out.front() == '\t')) {
    out.erase(out.begin());
  }
  while (!out.empty() && (out.back() == ' ' || out.back() == '\t')) {
    out.pop_back();
  }
  if (StartsWith(out, "grpc://")) {
    out.erase(0, 7);
  } else if (StartsWith(out, "https://")) {
    out.erase(0, 8);
  } else if (StartsWith(out, "http://")) {
    out.erase(0, 7);
  }
  // Drop any path/query if present (host:port only).
  const auto slash = out.find('/');
  if (slash != std::string::npos) {
    out.resize(slash);
  }
  return out;
}

std::string NormalizeMethodFullName(const std::string& method_full_name) {
  std::string name = method_full_name;
  while (!name.empty() && (name.front() == ' ' || name.front() == '\t' ||
                           name.front() == '/')) {
    name.erase(name.begin());
  }
  while (!name.empty() && (name.back() == ' ' || name.back() == '\t')) {
    name.pop_back();
  }
  for (char& c : name) {
    if (c == '/') {
      c = '.';
    }
  }
  return name;
}

GrpcSession::~GrpcSession() { cancel(); }

bool GrpcSession::connect(const std::string& target, bool use_tls,
                          bool verify_cert, const std::string& ssl_override,
                          std::string* err) {
#if !AUTOVIZ_ENABLE_GRPC
  (void)target;
  (void)use_tls;
  (void)verify_cert;
  (void)ssl_override;
  SetError(err, "gRPC disabled");
  return false;
#else
  const std::string host_port = NormalizeGrpcTarget(target);
  if (host_port.empty()) {
    SetError(err, "Empty gRPC target");
    return false;
  }

  cancel();

  std::shared_ptr<grpc::ChannelCredentials> creds;
  grpc::ChannelArguments args;
  if (!ssl_override.empty()) {
    args.SetSslTargetNameOverride(ssl_override);
  }

  if (!use_tls) {
    creds = grpc::InsecureChannelCredentials();
  } else if (verify_cert) {
    grpc::SslCredentialsOptions opts;
    creds = grpc::SslCredentials(opts);
  } else {
    // Prefer experimental TLS options that skip server cert verification.
    grpc::experimental::TlsChannelCredentialsOptions tls_opts;
    tls_opts.set_verify_server_certs(false);
    creds = grpc::experimental::TlsCredentials(tls_opts);
    if (!creds) {
      SetError(err,
               "TLS without cert verify unavailable; falling back to insecure "
               "credentials");
      creds = grpc::InsecureChannelCredentials();
    } else if (err != nullptr) {
      *err = "TLS enabled with certificate verification disabled";
    }
  }

  if (!creds) {
    SetError(err, "Failed to create channel credentials");
    return false;
  }

  channel_ = grpc::CreateCustomChannel(host_port, creds, args);
  if (!channel_) {
    SetError(err, "Failed to create gRPC channel");
    return false;
  }
  return true;
#endif
}

void GrpcSession::cancel() {
#if AUTOVIZ_ENABLE_GRPC
  std::lock_guard<std::mutex> lock(context_mu_);
  if (active_context_ != nullptr) {
    active_context_->TryCancel();
  }
#endif
}

#if AUTOVIZ_ENABLE_GRPC
void GrpcSession::setActiveContext(grpc::ClientContext* context) {
  std::lock_guard<std::mutex> lock(context_mu_);
  active_context_ = context;
}
#endif

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
