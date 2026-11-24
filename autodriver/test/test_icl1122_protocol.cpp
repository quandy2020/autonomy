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
 * @file test_icl1122_protocol.cpp
 * @brief Unit tests for EVB1122 / ICL1122 UART frame parsers.
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "autodriver/radar/icl1122/host_cfg.hpp"
#include "autodriver/radar/icl1122/protocol.hpp"
#include "autodriver/radar/icl1122/range_fft.hpp"
#include "autodriver/radar/backend_registry.hpp"

using autodriver::protocol::Icl1122McuParser;
using autodriver::protocol::Icl1122TargetParser;

namespace {

std::optional<autodriver::protocol::Icl1122TargetFrame> FeedAll(
    Icl1122TargetParser* parser, const std::vector<std::uint8_t>& bytes) {
  std::optional<autodriver::protocol::Icl1122TargetFrame> last;
  for (std::uint8_t b : bytes) {
    if (auto f = parser->Feed(b)) {
      last = std::move(f);
    }
  }
  return last;
}

}  // namespace

TEST(Icl1122Protocol, DecodesUm10035ExampleFrame) {
  // UM10035 §6.1.5 example (target1 only; slots 2/3 zero):
  // AA FF 03 00 0E 03 B1 86 10 00 40 01 | zeros 2–3 | 55 CC
  // x=-782 mm, y=+1713 mm, v=-16 cm/s, range_res=320 mm
  std::vector<std::uint8_t> frame = {
      0xAA, 0xFF, 0x03, 0x00, 0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00, 0x40, 0x01,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x55, 0xCC};
  Icl1122TargetParser parser;
  auto decoded = FeedAll(&parser, frame);
  ASSERT_TRUE(decoded.has_value());
  ASSERT_EQ(decoded->targets.size(), 3u);
  EXPECT_TRUE(decoded->targets[0].valid);
  EXPECT_FALSE(decoded->targets[1].valid);
  EXPECT_FALSE(decoded->targets[2].valid);
  EXPECT_NEAR(decoded->targets[0].x_m, -0.782, 1e-6);
  EXPECT_NEAR(decoded->targets[0].y_m, 1.713, 1e-6);
  EXPECT_NEAR(decoded->targets[0].velocity_mps, -0.16, 1e-6);
  EXPECT_NEAR(decoded->targets[0].range_resolution_m, 0.320, 1e-6);
}

TEST(Icl1122Protocol, ParsesMcuIclhWrap) {
  // JoySpace 24GMmwave: len = payload bytes only (here 2), then type, channel.
  std::vector<std::uint8_t> frame = {
      0x49, 0x43, 0x4C, 0x48, 0x02, 0x00, 0x00, 0x01, 0xAA, 0xBB,
      0x49, 0x43, 0x4C, 0x54};
  Icl1122McuParser parser;
  std::optional<autodriver::protocol::Icl1122McuFrame> got;
  for (std::uint8_t b : frame) {
    if (auto f = parser.Feed(b)) {
      got = std::move(f);
    }
  }
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->type, 0x00);
  EXPECT_EQ(got->channel, 0x01);
  ASSERT_EQ(got->payload.size(), 2u);
  EXPECT_EQ(got->payload[0], 0xAA);
  EXPECT_EQ(got->payload[1], 0xBB);
}

TEST(Icl1122Protocol, BackendRegistered) {
  auto& reg = autodriver::radar::RadarBackendRegistry::Instance();
  EXPECT_TRUE(reg.HasBackend("icl1122"));
  EXPECT_TRUE(reg.HasBackend("evb1122"));
  EXPECT_TRUE(reg.HasBackend("um10034"));
  EXPECT_TRUE(reg.HasBackend("iclegend"));
  autodriver::hardware::DriverParams params;
  params["device"] = "/dev/null";
  auto driver = reg.CreateDriver("icl1122", "radar/front", params);
  ASSERT_NE(driver, nullptr);
}

TEST(Icl1122HostCfg, BuildsStartCfgPacket) {
  const auto pkt = autodriver::protocol::BuildIcl1122HostPacket(
      autodriver::protocol::Icl1122HostCmd::kStartCfg, {0x02, 0x00});
  ASSERT_EQ(pkt.size(), 4u + 2u + 2u + 2u + 4u);
  EXPECT_EQ(pkt[0], 0xFD);
  EXPECT_EQ(pkt[1], 0xFC);
  EXPECT_EQ(pkt[2], 0xFB);
  EXPECT_EQ(pkt[3], 0xFA);
  // len = payload(2) + cmd(2) = 4
  EXPECT_EQ(pkt[4], 0x04);
  EXPECT_EQ(pkt[5], 0x00);
  // cmd 0x00FF LE
  EXPECT_EQ(pkt[6], 0xFF);
  EXPECT_EQ(pkt[7], 0x00);
  EXPECT_EQ(pkt[8], 0x02);
  EXPECT_EQ(pkt[9], 0x00);
  EXPECT_EQ(pkt[pkt.size() - 4], 0x04);
  EXPECT_EQ(pkt[pkt.size() - 1], 0x01);
}

TEST(Icl1122RangeFft, ParsesSubframeAndAssemblesPeaks) {
  // Minimal valid Range-FFT subframe: hdr dword + 1 complex (2×u16) + footer.
  // fft_field = fft_number + 1 = 2; chirp=0, channel=0, header=0xAA.
  // first BE dword: AA | ch=0 | chirp=0 | fft=2 → 0xAA000002
  // IQ: two BE uint16 = 0x0010, 0x0020 → wire bytes 00 10 00 20
  // checksum = 0x0010 + 0x0020 = 0x0030; frame=0; footer=0x55
  // last BE: checksum<<16 | frame<<12 | 0x55 → 0x00300055
  std::vector<std::uint8_t> payload = {
      0xAA, 0x00, 0x00, 0x02, 0x00, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00, 0x55};
  auto sub = autodriver::protocol::ParseIcl1122RangeFftPayload(payload);
  ASSERT_TRUE(sub.has_value());
  EXPECT_EQ(sub->rx_channel, 0u);
  EXPECT_EQ(sub->chirp_index, 0u);
  EXPECT_EQ(sub->frame_index, 0u);
  ASSERT_EQ(sub->iq_bytes.size(), 4u);

  // Geometry 1 chirp × 1 bin × 2 rx — feed rx0 then rx1 with same IQ.
  autodriver::protocol::Icl1122RangeFftAssembler asmblr;
  asmblr.SetConfig(/*chirp_number=*/1, /*range_bins=*/1, /*num_rx=*/2);
  EXPECT_FALSE(asmblr.Feed(*sub).has_value());
  autodriver::protocol::Icl1122RangeFftSubframe rx1 = *sub;
  rx1.rx_channel = 1;
  auto frame = asmblr.Feed(rx1);
  ASSERT_TRUE(frame.has_value());
  ASSERT_EQ(frame->bin_magnitude.size(), 1u);
  EXPECT_GT(frame->bin_magnitude[0], 0.f);
  ASSERT_GE(frame->assembled.size(), 16u);
  EXPECT_EQ(frame->assembled[0], 0x5A);

  std::vector<float> xs, ys, intens;
  autodriver::protocol::ExtractIcl1122RangePeaks(
      frame->bin_magnitude, /*range_resolution_m=*/0.5, /*peak_ratio=*/0.1f,
      /*skip_near_bins=*/0, &xs, &ys, &intens);
  ASSERT_EQ(xs.size(), 1u);
  EXPECT_NEAR(ys[0], 0.0f, 1e-6);
}


TEST(Icl1122RangeFft, DopplerVelocityFromChirpPhase) {
  // UM10034P §5.3 2DFFT: a target whose bin phase advances by a fixed step
  // per chirp has radial velocity lambda * step / (4*pi*T_chirp).
  constexpr std::uint32_t kBins = 16;
  constexpr std::uint32_t kRx = 2;
  constexpr std::uint32_t kChirps = 8;
  constexpr std::size_t kPerRx = kBins * 4;
  constexpr std::size_t kPerChirp = kPerRx * kRx;
  constexpr std::uint32_t kTargetBin = 5;
  constexpr double kChirpPeriod = 0.0025;
  constexpr double kLambda = 0.0125;
  constexpr double kPhaseStep = 0.6;
  std::vector<std::uint8_t> iq(kPerChirp * kChirps, 0);
  const auto put = [&iq](std::size_t offset, std::int16_t value) {
    std::memcpy(iq.data() + offset, &value, sizeof(value));
  };
  for (std::uint32_t c = 0; c < kChirps; ++c) {
    for (std::uint32_t rx = 0; rx < kRx; ++rx) {
      for (std::uint32_t b = 0; b < kBins; ++b) {
        double re = 500.0;  // static clutter everywhere
        double im = 0.0;
        if (b == kTargetBin) {
          re += 3000.0 * std::cos(kPhaseStep * c);
          im += 3000.0 * std::sin(kPhaseStep * c);
        }
        const std::size_t offset = c * kPerChirp + rx * kPerRx + b * 4;
        put(offset, static_cast<std::int16_t>(std::lround(re)));
        put(offset + 2, static_cast<std::int16_t>(std::lround(im)));
      }
    }
  }
  float velocity = 0.f;
  float quality = 0.f;
  ASSERT_TRUE(autodriver::protocol::EstimateIcl1122DopplerVelocity(
      iq, kTargetBin, kBins, kRx, kChirps, kChirpPeriod, kLambda, &velocity,
      &quality));
  const double expected = kLambda * kPhaseStep / (4.0 * M_PI * kChirpPeriod);
  // MTI and int16 quantisation bias the lag-1 estimate by a few percent.
  EXPECT_NEAR(velocity, expected, 0.05 * expected);
  EXPECT_GT(quality, 0.9f);
}

TEST(Icl1122RangeFft, PeakBinInterpolation) {
  const std::vector<float> magnitude = {1.f, 40.f, 100.f, 30.f, 1.f};
  EXPECT_NEAR(
      autodriver::protocol::InterpolateIcl1122PeakBin(magnitude, 2),
      2.0 - 0.5 * 10.0 / 130.0, 1e-6);
  EXPECT_DOUBLE_EQ(
      autodriver::protocol::InterpolateIcl1122PeakBin(magnitude, 0), 0.0);
}

TEST(Icl1122RangeFft, RangePeaksUseInterpolatedDistance) {
  std::vector<float> magnitude(16, 1.f);
  magnitude[10] = 100.f;
  magnitude[9] = 40.f;
  magnitude[11] = 30.f;
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> intensities;
  std::vector<std::size_t> bins;
  autodriver::protocol::ExtractIcl1122RangePeaks(magnitude, 0.15, 4.0f, 3, &xs,
                                                 &ys, &intensities, &bins);
  ASSERT_EQ(ys.size(), 1u);
  EXPECT_EQ(bins[0], 10u);
  EXPECT_NEAR(ys[0], (10.0 - 0.5 * 10.0 / 130.0) * 0.15, 1e-5);
}
