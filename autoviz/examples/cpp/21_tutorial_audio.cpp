/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kSr = 16000;
constexpr int kChunk = 1600;  // 100 ms @ 16 kHz
constexpr double kFreq = 440.0;

using autoviz::examples::MakeRawWriter;
using autoviz::examples::ParseRate;

std::string Base64Encode(const std::string& in) {
  static const char* kTbl =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((in.size() + 2) / 3) * 4);
  size_t i = 0;
  while (i + 2 < in.size()) {
    const uint32_t n = (static_cast<uint8_t>(in[i]) << 16) |
                       (static_cast<uint8_t>(in[i + 1]) << 8) |
                       static_cast<uint8_t>(in[i + 2]);
    out.push_back(kTbl[(n >> 18) & 63]);
    out.push_back(kTbl[(n >> 12) & 63]);
    out.push_back(kTbl[(n >> 6) & 63]);
    out.push_back(kTbl[n & 63]);
    i += 3;
  }
  if (i < in.size()) {
    uint32_t n = static_cast<uint8_t>(in[i]) << 16;
    out.push_back(kTbl[(n >> 18) & 63]);
    if (i + 1 < in.size()) {
      n |= static_cast<uint8_t>(in[i + 1]) << 8;
      out.push_back(kTbl[(n >> 12) & 63]);
      out.push_back(kTbl[(n >> 6) & 63]);
      out.push_back('=');
    } else {
      out.push_back(kTbl[(n >> 12) & 63]);
      out.push_back('=');
      out.push_back('=');
    }
  }
  return out;
}

std::string MakePcm(double* phase, int n, double freq = kFreq) {
  std::string buf;
  buf.resize(static_cast<size_t>(n) * 2);
  const double step = 2.0 * M_PI * freq / kSr;
  for (int i = 0; i < n; ++i) {
    int s = static_cast<int>(8000.0 * std::sin(*phase));
    if (s > 32767) s = 32767;
    if (s < -32767) s = -32767;
    const int16_t sample = static_cast<int16_t>(s);
    std::memcpy(&buf[static_cast<size_t>(i) * 2], &sample, 2);
    *phase += step;
  }
  return buf;
}

std::string MakeMsg(const std::string& pcm) {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  const int64_t sec = ns / 1000000000LL;
  const int64_t nsec = ns % 1000000000LL;
  return std::string("{\"timestamp\":{\"sec\":") + std::to_string(sec) +
         ",\"nsec\":" + std::to_string(nsec) +
         "},\"format\":\"pcm-s16\",\"sample_rate\":" + std::to_string(kSr) +
         ",\"number_of_channels\":1,\"data\":\"" + Base64Encode(pcm) + "\"}";
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/audio");
  auto writer = MakeRawWriter(node, "/fake/audio", "foxglove.RawAudio");

  autolink::Rate rate(rate_hz);
  std::cout << "audio @ " << rate_hz << " Hz → /fake/audio (pcm-s16 " << kSr
            << " Hz)\n";

  double phase = 0.0;
  while (autolink::OK()) {
    const std::string pcm = MakePcm(&phase, kChunk);
    auto raw =
        std::make_shared<autolink::message::RawMessage>(MakeMsg(pcm));
    writer->Write(raw);
    rate.Sleep();
  }
  return 0;
}
