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
 * @file range_fft.cpp
 * @brief Range-FFT subframe parse + frame assemble (implementation).
 */

#include "autodriver/radar/icl1122/range_fft.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace autodriver {
namespace protocol {
namespace {

constexpr std::uint8_t kHdr = 0xAA;
constexpr std::uint8_t kFtr = 0x55;

std::uint32_t BeDword(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) |
         (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) |
         static_cast<std::uint32_t>(p[3]);
}

std::uint32_t DecodeChirpIndex(std::uint32_t dword) {
  const std::uint32_t raw = (dword >> 11) & 0x1FFu;
  return ((raw & 0x100u) | ((raw & 0x1FEu) >> 1));
}

void PushU32Le(std::vector<std::uint8_t>* out, std::uint32_t v) {
  out->push_back(static_cast<std::uint8_t>(v & 0xFF));
  out->push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
  out->push_back(static_cast<std::uint8_t>((v >> 16) & 0xFF));
  out->push_back(static_cast<std::uint8_t>((v >> 24) & 0xFF));
}

std::vector<float> MeanBinMagnitude(const std::vector<std::uint8_t>& iq,
                                    std::uint32_t range_bins,
                                    std::uint32_t num_rx,
                                    std::uint32_t chirp_number) {
  std::vector<float> mag(range_bins, 0.f);
  if (range_bins == 0 || num_rx == 0 || chirp_number == 0) {
    return mag;
  }
  const std::size_t per_rx = static_cast<std::size_t>(range_bins) * 4u;  // I,Q
  const std::size_t per_chirp = per_rx * num_rx;
  const std::size_t need = per_chirp * chirp_number;
  if (iq.size() < need) {
    return mag;
  }
  const float inv =
      1.f / static_cast<float>(num_rx * chirp_number);
  for (std::uint32_t c = 0; c < chirp_number; ++c) {
    for (std::uint32_t rx = 0; rx < num_rx; ++rx) {
      const std::size_t base = c * per_chirp + rx * per_rx;
      for (std::uint32_t b = 0; b < range_bins; ++b) {
        const std::size_t o = base + static_cast<std::size_t>(b) * 4u;
        std::int16_t re = 0;
        std::int16_t im = 0;
        std::memcpy(&re, iq.data() + o, sizeof(re));
        std::memcpy(&im, iq.data() + o + 2, sizeof(im));
        mag[b] += std::hypot(static_cast<float>(re), static_cast<float>(im));
      }
    }
  }
  for (float& m : mag) {
    m *= inv;
  }
  return mag;
}

}  // namespace

std::optional<Icl1122RangeFftSubframe> ParseIcl1122RangeFftPayload(
    const std::vector<std::uint8_t>& payload) {
  if (payload.size() < 8 || (payload.size() % 4) != 0) {
    return std::nullopt;
  }
  const std::uint32_t first = BeDword(payload.data());
  if (((first >> 24) & 0xFF) != kHdr) {
    return std::nullopt;
  }
  const std::uint32_t rx = (first >> 23) & 0x1u;
  const std::uint32_t chirp = DecodeChirpIndex(first);
  const std::uint32_t fft_field = first & 0x7FFu;
  if (fft_field < 1) {
    return std::nullopt;
  }
  // JoySpace: fftNumber_ = field - 1; last dword is footer (not FFT sample).
  const std::uint32_t fft_number = fft_field - 1;
  if (payload.size() != static_cast<std::size_t>(fft_number) * 4u + 8u) {
    return std::nullopt;
  }
  const std::uint32_t last = BeDword(payload.data() + payload.size() - 4);
  if ((last & 0xFFu) != kFtr) {
    return std::nullopt;
  }
  const std::uint32_t checksum = (last >> 16) & 0xFFFFu;
  const std::uint32_t frame_index = (last >> 12) & 0xFu;

  // Wire words → host uint16 BE pairs, then checksum over those uint16s.
  std::vector<std::uint16_t> words;
  words.reserve(static_cast<std::size_t>(fft_number) * 2u);
  std::uint32_t sum = 0;
  for (std::uint32_t i = 0; i < fft_number * 2u; ++i) {
    const std::uint16_t w = static_cast<std::uint16_t>(
        (payload[4 + i * 2] << 8) | payload[4 + i * 2 + 1]);
    words.push_back(w);
    sum = static_cast<std::uint32_t>(static_cast<std::uint16_t>(sum + w));
  }
  if (sum != checksum) {
    return std::nullopt;
  }

  Icl1122RangeFftSubframe out;
  out.rx_channel = rx;
  out.chirp_index = chirp;
  out.frame_index = frame_index;
  out.iq_bytes.resize(words.size() * sizeof(std::uint16_t));
  // Store as native LE int16 pairs for later hypot().
  for (std::size_t i = 0; i < words.size(); ++i) {
    const auto v = static_cast<std::int16_t>(words[i]);
    std::memcpy(out.iq_bytes.data() + i * 2, &v, sizeof(v));
  }
  return out;
}

void Icl1122RangeFftAssembler::SetConfig(std::uint32_t chirp_number,
                                         std::uint32_t range_bins,
                                         std::uint32_t num_rx) {
  chirp_number_ = chirp_number == 0 ? 1 : chirp_number;
  range_bins_ = range_bins == 0 ? 1 : range_bins;
  num_rx_ = num_rx == 0 ? 1 : num_rx;
  Reset();
}

void Icl1122RangeFftAssembler::Reset() {
  state_ = State::kSearchStart;
  frame_index_ = 0;
  channels_got_ = 0;
  chirp_iq_.clear();
  rx_ready_.assign(num_rx_, false);
  assembled_iq_.clear();
}

std::optional<Icl1122RangeFftFrame> Icl1122RangeFftAssembler::Feed(
    const Icl1122RangeFftSubframe& sub) {
  const std::size_t expect_iq = static_cast<std::size_t>(range_bins_) * 4u;
  if (sub.iq_bytes.size() < expect_iq || sub.rx_channel >= num_rx_) {
    return std::nullopt;
  }

  auto finish_chirp_if_ready = [&]() {
    bool all = true;
    for (bool r : rx_ready_) {
      if (!r) {
        all = false;
        break;
      }
    }
    if (!all) {
      return;
    }
    assembled_iq_.insert(assembled_iq_.end(), chirp_iq_.begin(),
                         chirp_iq_.end());
    channels_got_ += num_rx_;
    chirp_iq_.clear();
    chirp_iq_.resize(expect_iq * num_rx_, 0);
    std::fill(rx_ready_.begin(), rx_ready_.end(), false);
  };

  auto accept_channel = [&](const Icl1122RangeFftSubframe& s) {
    if (rx_ready_[s.rx_channel]) {
      Reset();
      return false;
    }
    if (chirp_iq_.empty()) {
      chirp_iq_.assign(expect_iq * num_rx_, 0);
    }
    std::memcpy(chirp_iq_.data() + s.rx_channel * expect_iq, s.iq_bytes.data(),
                expect_iq);
    rx_ready_[s.rx_channel] = true;
    finish_chirp_if_ready();
    return true;
  };

  switch (state_) {
    case State::kSearchStart:
      // JoySpace: start-of-frame when chirpIndex == 0.
      if (sub.chirp_index != 0) {
        return std::nullopt;
      }
      frame_index_ = sub.frame_index;
      channels_got_ = 0;
      assembled_iq_.clear();
      chirp_iq_.clear();
      rx_ready_.assign(num_rx_, false);
      if (!accept_channel(sub)) {
        return std::nullopt;
      }
      state_ = State::kCollect;
      break;

    case State::kCollect:
      if (sub.frame_index != frame_index_) {
        Reset();
        return Feed(sub);
      }
      if (!accept_channel(sub)) {
        return std::nullopt;
      }
      break;
  }

  if (channels_got_ < chirp_number_ * num_rx_) {
    return std::nullopt;
  }

  Icl1122RangeFftFrame frame;
  frame.frame_index = frame_index_;
  frame.range_bins = range_bins_;
  frame.num_rx = num_rx_;
  frame.chirp_number = chirp_number_;
  frame.bin_magnitude =
      MeanBinMagnitude(assembled_iq_, range_bins_, num_rx_, chirp_number_);
  frame.iq_bytes = assembled_iq_;
  frame.assembled.clear();
  PushU32Le(&frame.assembled, 0x5A5A5A5Au);
  PushU32Le(&frame.assembled, frame_index_);
  PushU32Le(&frame.assembled, range_bins_ * num_rx_ * chirp_number_);
  frame.assembled.insert(frame.assembled.end(), assembled_iq_.begin(),
                         assembled_iq_.end());
  PushU32Le(&frame.assembled, 0xA5A5A5A5u);
  Reset();
  return frame;
}

void ExtractIcl1122RangePeaks(const std::vector<float>& bin_magnitude,
                              double range_resolution_m, float peak_ratio,
                              std::size_t skip_near_bins,
                              std::vector<float>* xs, std::vector<float>* ys,
                              std::vector<float>* intensities,
                              std::vector<std::size_t>* bins) {
  if (xs == nullptr || ys == nullptr || intensities == nullptr) {
    return;
  }
  xs->clear();
  ys->clear();
  intensities->clear();
  if (bins != nullptr) {
    bins->clear();
  }
  if (bin_magnitude.empty() || skip_near_bins >= bin_magnitude.size()) {
    return;
  }
  const std::size_t n = bin_magnitude.size();
  const std::size_t usable = n - skip_near_bins;
  if (usable == 0) {
    return;
  }

  // Tiny frames (unit tests): emit any positive bin without CFAR.
  if (usable < 3) {
    for (std::size_t i = skip_near_bins; i < n; ++i) {
      if (bin_magnitude[i] <= 0.f) {
        continue;
      }
      xs->push_back(0.f);
      ys->push_back(static_cast<float>(InterpolateIcl1122PeakBin(
                                       bin_magnitude, i) * range_resolution_m));
      intensities->push_back(bin_magnitude[i]);
      if (bins != nullptr) {
        bins->push_back(i);
      }
    }
    return;
  }

  // Noise from the farthest third — near TX leakage must not set the bar.
  const std::size_t noise_start =
      skip_near_bins + (usable * 2) / 3;
  float noise_sum = 0.f;
  std::size_t noise_count = 0;
  for (std::size_t i = noise_start; i < n; ++i) {
    noise_sum += bin_magnitude[i];
    ++noise_count;
  }
  const float noise =
      noise_count > 0 ? (noise_sum / static_cast<float>(noise_count)) : 0.f;
  // peak_ratio acts as CFAR multiplier over the far-bin noise floor.
  const float cfar = peak_ratio > 1.f ? peak_ratio : 4.f;
  const float thr = std::max(noise * cfar, 1.f);

  for (std::size_t i = skip_near_bins; i < n; ++i) {
    if (bin_magnitude[i] < thr) {
      continue;
    }
    const float left =
        (i == skip_near_bins) ? 0.f : bin_magnitude[i - 1];
    const float right = (i + 1 >= n) ? 0.f : bin_magnitude[i + 1];
    if (bin_magnitude[i] <= left || bin_magnitude[i] <= right) {
      continue;
    }
    xs->push_back(0.f);
    ys->push_back(static_cast<float>(InterpolateIcl1122PeakBin(
                                     bin_magnitude, i) * range_resolution_m));
    intensities->push_back(bin_magnitude[i]);
    if (bins != nullptr) {
      bins->push_back(i);
    }
  }
}

void KeepTopIcl1122Peaks(std::size_t max_peaks, float min_peak_frac,
                         std::vector<float>* xs, std::vector<float>* ys,
                         std::vector<float>* intensities,
                         std::vector<std::size_t>* bins) {
  if (xs == nullptr || ys == nullptr || intensities == nullptr) {
    return;
  }
  const std::size_t n = intensities->size();
  if (n == 0 || (xs->size() != n || ys->size() != n) ||
      (bins != nullptr && bins->size() != n)) {
    return;
  }

  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) {
    order[i] = i;
  }
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return (*intensities)[a] > (*intensities)[b];
  });

  const float peak0 = (*intensities)[order[0]];
  const float floor_thr =
      (min_peak_frac > 0.f) ? (peak0 * min_peak_frac) : 0.f;
  const std::size_t keep_cap =
      (max_peaks == 0) ? n : std::min(max_peaks, n);

  std::vector<float> ox;
  std::vector<float> oy;
  std::vector<float> oi;
  std::vector<std::size_t> ob;
  ox.reserve(keep_cap);
  oy.reserve(keep_cap);
  oi.reserve(keep_cap);
  if (bins != nullptr) {
    ob.reserve(keep_cap);
  }
  for (std::size_t k = 0; k < n && ox.size() < keep_cap; ++k) {
    const std::size_t i = order[k];
    if ((*intensities)[i] < floor_thr) {
      break;
    }
    ox.push_back((*xs)[i]);
    oy.push_back((*ys)[i]);
    oi.push_back((*intensities)[i]);
    if (bins != nullptr) {
      ob.push_back((*bins)[i]);
    }
  }
  *xs = std::move(ox);
  *ys = std::move(oy);
  *intensities = std::move(oi);
  if (bins != nullptr) {
    *bins = std::move(ob);
  }
}

float EstimateIcl1122AzimuthFromIq(const std::vector<std::uint8_t>& iq_bytes,
                                   std::size_t bin, std::uint32_t range_bins,
                                   std::uint32_t num_rx,
                                   std::uint32_t chirp_number,
                                   double rx_spacing_m, double wavelength_m) {
  if (num_rx < 2 || range_bins == 0 || chirp_number == 0 ||
      bin >= range_bins || rx_spacing_m <= 0.0 || wavelength_m <= 0.0) {
    return 0.f;
  }
  const std::size_t per_rx = static_cast<std::size_t>(range_bins) * 4u;
  const std::size_t per_chirp = per_rx * num_rx;
  const std::size_t need = per_chirp * chirp_number;
  if (iq_bytes.size() < need) {
    return 0.f;
  }

  double sum_re = 0.0;
  double sum_im = 0.0;
  for (std::uint32_t c = 0; c < chirp_number; ++c) {
    const std::size_t o0 =
        c * per_chirp + 0u * per_rx + bin * 4u;
    const std::size_t o1 =
        c * per_chirp + 1u * per_rx + bin * 4u;
    std::int16_t re0 = 0;
    std::int16_t im0 = 0;
    std::int16_t re1 = 0;
    std::int16_t im1 = 0;
    std::memcpy(&re0, iq_bytes.data() + o0, sizeof(re0));
    std::memcpy(&im0, iq_bytes.data() + o0 + 2, sizeof(im0));
    std::memcpy(&re1, iq_bytes.data() + o1, sizeof(re1));
    std::memcpy(&im1, iq_bytes.data() + o1 + 2, sizeof(im1));
    // RX1 * conj(RX0)
    const double re =
        static_cast<double>(re1) * re0 + static_cast<double>(im1) * im0;
    const double im =
        static_cast<double>(im1) * re0 - static_cast<double>(re1) * im0;
    sum_re += re;
    sum_im += im;
  }
  const double phase = std::atan2(sum_im, sum_re);
  const double arg =
      phase * wavelength_m / (2.0 * 3.14159265358979323846 * rx_spacing_m);
  const double clamped = std::max(-1.0, std::min(1.0, arg));
  return static_cast<float>(std::asin(clamped));
}

double InterpolateIcl1122PeakBin(const std::vector<float>& bin_magnitude,
                                 std::size_t bin) {
  if (bin_magnitude.size() < 3 || bin == 0 ||
      bin + 1 >= bin_magnitude.size()) {
    return static_cast<double>(bin);
  }
  const double previous = bin_magnitude[bin - 1];
  const double current = bin_magnitude[bin];
  const double next = bin_magnitude[bin + 1];
  const double denominator = previous - 2.0 * current + next;
  if (std::fabs(denominator) < 1e-12) {
    return static_cast<double>(bin);
  }
  const double delta = 0.5 * (previous - next) / denominator;
  return static_cast<double>(bin) + std::clamp(delta, -0.5, 0.5);
}

bool EstimateIcl1122DopplerVelocity(const std::vector<std::uint8_t>& iq_bytes,
                                    std::size_t bin, std::uint32_t range_bins,
                                    std::uint32_t num_rx,
                                    std::uint32_t chirp_number,
                                    double chirp_period_s,
                                    double wavelength_m,
                                    float* velocity_mps, float* quality) {
  constexpr double kPi = 3.14159265358979323846;
  if (velocity_mps == nullptr || range_bins == 0 || num_rx == 0 ||
      chirp_number < 3 || bin >= range_bins || !(chirp_period_s > 0.0) ||
      !(wavelength_m > 0.0)) {
    return false;
  }
  const std::size_t per_rx = static_cast<std::size_t>(range_bins) * 4u;
  const std::size_t per_chirp = per_rx * num_rx;
  if (iq_bytes.size() < per_chirp * chirp_number) {
    return false;
  }
  double sum_re = 0.0;
  double sum_im = 0.0;
  double magnitude_sum = 0.0;
  for (std::uint32_t rx = 0; rx < num_rx; ++rx) {
    // Mean complex value across chirps = static clutter for this bin/RX.
    double mean_re = 0.0;
    double mean_im = 0.0;
    for (std::uint32_t c = 0; c < chirp_number; ++c) {
      const std::size_t offset =
          c * per_chirp + rx * per_rx + static_cast<std::size_t>(bin) * 4u;
      std::int16_t re = 0;
      std::int16_t im = 0;
      std::memcpy(&re, iq_bytes.data() + offset, sizeof(re));
      std::memcpy(&im, iq_bytes.data() + offset + 2, sizeof(im));
      mean_re += static_cast<double>(re);
      mean_im += static_cast<double>(im);
    }
    mean_re /= static_cast<double>(chirp_number);
    mean_im /= static_cast<double>(chirp_number);
    double previous_re = 0.0;
    double previous_im = 0.0;
    bool have_previous = false;
    for (std::uint32_t c = 0; c < chirp_number; ++c) {
      const std::size_t offset =
          c * per_chirp + rx * per_rx + static_cast<std::size_t>(bin) * 4u;
      std::int16_t re = 0;
      std::int16_t im = 0;
      std::memcpy(&re, iq_bytes.data() + offset, sizeof(re));
      std::memcpy(&im, iq_bytes.data() + offset + 2, sizeof(im));
      const double current_re = static_cast<double>(re) - mean_re;
      const double current_im = static_cast<double>(im) - mean_im;
      if (have_previous) {
        // current * conj(previous): angle is the chirp-to-chirp phase step.
        sum_re += current_re * previous_re + current_im * previous_im;
        sum_im += current_im * previous_re - current_re * previous_im;
        magnitude_sum += std::hypot(current_re, current_im) *
                         std::hypot(previous_re, previous_im);
      }
      previous_re = current_re;
      previous_im = current_im;
      have_previous = true;
    }
  }
  if (magnitude_sum <= 0.0) {
    return false;
  }
  const double phase = std::atan2(sum_im, sum_re);
  *velocity_mps = static_cast<float>(wavelength_m * phase /
                                     (4.0 * kPi * chirp_period_s));
  if (quality != nullptr) {
    *quality = static_cast<float>(std::hypot(sum_re, sum_im) / magnitude_sum);
  }
  return true;
}

void ExtractIcl1122RangeSpectrum(const std::vector<float>& bin_magnitude,
                                 double range_resolution_m,
                                 std::size_t skip_near_bins,
                                 std::vector<float>* xs, std::vector<float>* ys,
                                 std::vector<float>* intensities) {
  if (xs == nullptr || ys == nullptr || intensities == nullptr) {
    return;
  }
  xs->clear();
  ys->clear();
  intensities->clear();
  for (std::size_t i = skip_near_bins; i < bin_magnitude.size(); ++i) {
    if (!(bin_magnitude[i] > 0.f)) {
      continue;
    }
    xs->push_back(0.f);
    ys->push_back(static_cast<float>(static_cast<double>(i) * range_resolution_m));
    intensities->push_back(bin_magnitude[i]);
  }
}

}  // namespace protocol
}  // namespace autodriver
