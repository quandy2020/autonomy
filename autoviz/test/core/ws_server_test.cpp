/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ws_server_test.cpp
 * @brief Loopback integration test: handshake, subscribe, binary broadcast.
 *
 * Starts a real WsServer on an ephemeral localhost port, performs a WebSocket
 * upgrade, subscribes to one channel and verifies the binary frame the server
 * broadcasts (header layout is shared with ws_frame.hpp).
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "autoviz/bridge/ws_frame.hpp"
#include "autoviz/bridge/ws_server.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

int FreeLocalPort() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return 0;
  }
  socklen_t len = sizeof(addr);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
    ::close(fd);
    return 0;
  }
  ::close(fd);
  return ntohs(addr.sin_port);
}

bool SendAll(int fd, const std::string& data) {
  std::size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, 0);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

bool RecvExact(int fd, void* out, std::size_t size, int timeout_ms = 3000) {
  auto* bytes = static_cast<std::uint8_t*>(out);
  std::size_t got = 0;
  while (got < size) {
    pollfd pfd{fd, POLLIN, 0};
    if (::poll(&pfd, 1, timeout_ms) <= 0) {
      return false;
    }
    const ssize_t n = ::recv(fd, bytes + got, size - got, 0);
    if (n <= 0) {
      return false;
    }
    got += static_cast<std::size_t>(n);
  }
  return true;
}

bool ReadFrame(int fd, std::uint8_t* opcode, std::string* payload,
               int timeout_ms = 3000) {
  std::uint8_t header[2] = {0, 0};
  if (!RecvExact(fd, header, 2, timeout_ms)) {
    return false;
  }
  *opcode = header[0] & 0x0f;
  std::uint64_t len = header[1] & 0x7f;
  if (len == 126) {
    std::uint8_t ext[2];
    if (!RecvExact(fd, ext, 2, timeout_ms)) {
      return false;
    }
    len = (static_cast<std::uint64_t>(ext[0]) << 8) | ext[1];
  } else if (len == 127) {
    std::uint8_t ext[8];
    if (!RecvExact(fd, ext, 8, timeout_ms)) {
      return false;
    }
    len = 0;
    for (int i = 0; i < 8; ++i) {
      len = (len << 8) | ext[i];
    }
  }
  payload->assign(static_cast<std::size_t>(len), '\0');
  if (len > 0 &&
      !RecvExact(fd, payload->data(), static_cast<std::size_t>(len),
                 timeout_ms)) {
    return false;
  }
  return true;
}

bool SendMaskedText(int fd, const std::string& text) {
  std::string frame;
  frame.push_back(static_cast<char>(0x81));  // FIN + text
  if (text.size() < 126) {
    frame.push_back(static_cast<char>(0x80 | text.size()));
  } else {
    frame.push_back(static_cast<char>(0x80 | 126));
    frame.push_back(static_cast<char>((text.size() >> 8) & 0xff));
    frame.push_back(static_cast<char>(text.size() & 0xff));
  }
  const std::uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
  frame.append(reinterpret_cast<const char*>(mask), 4);
  for (std::size_t i = 0; i < text.size(); ++i) {
    frame.push_back(static_cast<char>(text[i] ^ mask[i % 4]));
  }
  return SendAll(fd, frame);
}

}  // namespace

int main() {
  const int port = FreeLocalPort();
  if (port == 0) {
    std::fprintf(stderr, "FAIL: no free localhost port\n");
    return 1;
  }

  autoviz::bridge::WsServerOptions options;
  options.enabled = true;
  options.host = "127.0.0.1";
  options.port = static_cast<std::uint16_t>(port);
  options.server_name = "autoviz-test";

  autoviz::bridge::WsServer server(options);
  Check(server.Start(), "server starts on localhost");

  autoviz::bridge::ChannelInfo channel;
  channel.id = 0;
  channel.topic = "/test_topic";
  channel.message_type = "test.Type";
  const auto ids = server.AddChannels({channel});
  Check(ids.size() == 1 && ids.front() == 1, "first channel id is 1");

  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  Check(fd >= 0, "client socket opens");
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  Check(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0,
        "client connects");

  const std::string request =
      "GET / HTTP/1.1\r\n"
      "Host: 127.0.0.1\r\n"
      "Upgrade: websocket\r\n"
      "Connection: Upgrade\r\n"
      "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
      "Sec-WebSocket-Version: 13\r\n\r\n";
  Check(SendAll(fd, request), "handshake request sent");

  std::string response;
  char buffer[1] = {0};
  while (response.find("\r\n\r\n") == std::string::npos) {
    pollfd pfd{fd, POLLIN, 0};
    if (::poll(&pfd, 1, 3000) <= 0) {
      break;
    }
    if (::recv(fd, buffer, 1, 0) != 1) {
      break;
    }
    response.push_back(buffer[0]);
    if (response.size() > 4096) {
      break;
    }
  }
  Check(response.find("101 Switching Protocols") != std::string::npos,
        "server accepts the WebSocket upgrade");
  Check(response.find("Sec-WebSocket-Accept:") != std::string::npos,
        "server returns Sec-WebSocket-Accept");

  Check(SendMaskedText(fd, "{\"op\":\"subscribe\",\"channelId\":1}"),
        "subscribe frame sent");

  // The subscribe text frame is handled on the civetweb thread; give it a
  // moment, then broadcast a few times while draining frames.
  const std::string payload = "abc";
  bool saw_hello = false;
  bool saw_payload = false;
  for (int attempt = 0; attempt < 6 && !saw_payload; ++attempt) {
    if (attempt == 0) {
      usleep(200000);
    }
    server.BroadcastMessage(
        1, 42, reinterpret_cast<const std::uint8_t*>(payload.data()),
        payload.size());
    for (int i = 0; i < 8; ++i) {
      std::uint8_t opcode = 0;
      std::string frame;
      if (!ReadFrame(fd, &opcode, &frame, 500)) {
        break;
      }
      if (opcode == 0x1) {
        if (frame.find("\"hello\"") != std::string::npos) {
          saw_hello = true;
        }
        continue;
      }
      if (opcode != 0x2) {
        continue;
      }
      autoviz::bridge::DecodedMessageFrame decoded;
      if (!autoviz::bridge::DecodeMessageFrame(frame, &decoded)) {
        continue;
      }
      if (decoded.channel_id == 1 && decoded.payload == payload) {
        saw_payload = true;
        break;
      }
    }
  }
  Check(saw_hello, "server greets the client (hello frame)");
  Check(saw_payload, "subscribed channel receives the binary payload");

  ::close(fd);
  server.Stop();
  server.Stop();

  if (g_failures != 0) {
    std::fprintf(stderr, "%d ws server test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("ws server tests passed\n");
  return 0;
}

