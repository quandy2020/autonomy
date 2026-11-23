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
 * @file range_fft.hpp
 * @brief Parse JoySpace Range-FFT MCU payloads and assemble one frame.
 *
 * Inner DSP subframe (big-endian dwords on the wire, see 24GMmwave RangeData):
 *   dword0: header 0xAA | channel | chirp | fft_count
 *   fft words…
 *   last dword: checksum | frame_index | footer 0x55
 *
 * Assembled host frame (same as JoySpace rangebin output files):
 *   0x5A5A5A5A | frameIndex | Ncomplex | IQ… | 0xA5A5A5A5
 * Ncomplex = range_bins * num_rx * chirp_number (default 8*2*16).
 */

#ifndef AUTODRIVER_RADAR_ICL1122_RANGE_FFT_HPP_
#define AUTODRIVER_RADAR_ICL1122_RANGE_FFT_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace autodriver {
namespace protocol {

/** @brief One decoded Range-FFT subframe from an ICLH payload. */
struct Icl1122RangeFftSubframe {
  std::uint32_t rx_channel{0};
  std::uint32_t chirp_index{0};
  std::uint32_t frame_index{0};
  // Host-endian uint16 pairs (I,Q,I,Q…) for fft_number complex bins.
  std::vector<std::uint8_t> iq_bytes;
};

/**
 * @brief Parse one MCU payload as a Range-FFT subframe.
 * @param[in] payload Bytes inside ICLH…ICLT (not including type/channel).
 * @return Subframe when header/footer/checksum match; otherwise nullopt.
 */
std::optional<Icl1122RangeFftSubframe> ParseIcl1122RangeFftPayload(
    const std::vector<std::uint8_t>& payload);

/** @brief Completed Range-FFT frame ready for peak → PointCloud. */
struct Icl1122RangeFftFrame {
  std::uint32_t frame_index{0};
  // Mean |IQ| per range bin (size == range_bins), averaged over chirp×rx.
  std::vector<float> bin_magnitude;
  // Concatenated chirp×rx complex bins (I,Q int16 LE), size =
  // chirp_number * num_rx * range_bins * 4. Used for dual-RX angle.
  std::vector<std::uint8_t> iq_bytes;
  std::uint32_t range_bins{0};
  std::uint32_t num_rx{0};
  std::uint32_t chirp_number{0};
  // Raw assembled host bin (header + iq + footer).
  std::vector<std::uint8_t> assembled;
};

/**
 * @class autodriver::protocol::Icl1122RangeFftAssembler
 * @brief Collect chirp×rx subframes into one Icl1122RangeFftFrame.
 */
class Icl1122RangeFftAssembler {
 public:
  /**
   * @brief Configure expected geometry (JoySpace CLI args).
   * @param[in] chirp_number Number of chirps per frame (e.g. 16).
   * @param[in] range_bins FFT bins per RX (e.g. 8).
   * @param[in] num_rx RX channels (EVB1122 = 2).
   */
  void SetConfig(std::uint32_t chirp_number, std::uint32_t range_bins,
                 std::uint32_t num_rx = 2);

  /** @brief Drop partial assembly state. */
  void Reset();

  /**
   * @brief Feed one parsed subframe.
   * @param[in] sub Decoded Range-FFT subframe.
   * @return Complete frame when chirp_number * num_rx channels collected.
   */
  std::optional<Icl1122RangeFftFrame> Feed(const Icl1122RangeFftSubframe& sub);

 private:
  enum class State : std::uint8_t { kSearchStart, kCollect };

  State state_{State::kSearchStart};
  std::uint32_t chirp_number_{16};
  std::uint32_t range_bins_{8};
  std::uint32_t num_rx_{2};
  std::uint32_t frame_index_{0};
  std::uint32_t channels_got_{0};
  std::vector<std::uint8_t> chirp_iq_;  // accumulating one chirp (rx0|rx1)
  std::vector<bool> rx_ready_;
  std::vector<std::uint8_t> assembled_iq_;  // all chirps concatenated
};

/**
 * @brief Pick range-bin peaks above a relative threshold.
 * @param[in] bin_magnitude Per-bin mean |IQ|.
 * @param[in] range_resolution_m Meters per bin along board normal (Y).
 * @param[in] peak_ratio CFAR multiplier: thr = far_bin_noise * peak_ratio.
 * @param[in] skip_near_bins Skip the first N bins (DC / TX leakage).
 * @param[out] xs Lateral (always 0 for 1D range).
 * @param[out] ys Range along Y.
 * @param[out] intensities Magnitude.
 */
void ExtractIcl1122RangePeaks(const std::vector<float>& bin_magnitude,
                              double range_resolution_m, float peak_ratio,
                              std::size_t skip_near_bins,
                              std::vector<float>* xs, std::vector<float>* ys,
                              std::vector<float>* intensities,
                              std::vector<std::size_t>* bins = nullptr);

/**
 * @brief Keep strongest peaks; drop multipath/wall ghosts.
 * @param[in] max_peaks Max returns (1 = single person). 0 = unlimited.
 * @param[in] min_peak_frac Drop peaks below max_intensity * frac (0 = off).
 */
void KeepTopIcl1122Peaks(std::size_t max_peaks, float min_peak_frac,
                         std::vector<float>* xs, std::vector<float>* ys,
                         std::vector<float>* intensities,
                         std::vector<std::size_t>* bins);

/**
 * @brief Dual-RX phase → azimuth (rad) at one range bin (1T2R AoA).
 * @param[in] iq_bytes Frame IQ layout: chirp × (rx0 bins | rx1 bins) …
 * @param[in] bin Peak range-bin index.
 * @param[in] range_bins / num_rx / chirp_number Geometry.
 * @param[in] rx_spacing_m RX1–RX2 baseline (m); ~λ/2 ≈ 0.00625 at 24 GHz.
 * @param[in] wavelength_m Carrier wavelength (m); default 24 GHz.
 * @return Azimuth rad (atan2 lateral, +X); 0 if IQ unavailable.
 */
float EstimateIcl1122AzimuthFromIq(const std::vector<std::uint8_t>& iq_bytes,
                                   std::size_t bin, std::uint32_t range_bins,
                                   std::uint32_t num_rx,
                                   std::uint32_t chirp_number,
                                   double rx_spacing_m = 0.00625,
                                   double wavelength_m = 0.0125);

/**
 * @brief Sub-bin range refinement from three neighbouring magnitudes.
 *
 * UM10034P §4.4/§5.4 report the range resolution as c/(2B) (0.63 m for the
 * manual's 245 MHz GUI example). Three-point parabolic interpolation recovers
 * a fraction of one bin when the peak is well sampled, so a person's distance
 * does not jump by a whole 0.15-0.63 m step.
 *
 * @param[in] bin_magnitude Per-bin mean |IQ| (size == range bins).
 * @param[in] bin Integer peak index.
 * @return Fractional bin index clamped to [bin - 0.5, bin + 0.5]; the integer
 *         index when the neighbourhood is flat or at the edge.
 */
double InterpolateIcl1122PeakBin(const std::vector<float>& bin_magnitude,
                                 std::size_t bin);

/**
 * @brief Chirp slow-time phase progression -> radial velocity (2DFFT).
 *
 * UM10034P §5.3: every Range-FFT subframe carries one chirp of one RX, so the
 * assembled frame is a chirp x RX x bin complex cube. For a moving target the
 * bin phase advances by 4*pi*v*T_chirp/lambda per chirp. The per-RX complex
 * mean is removed first (MTI) so static clutter does not bias the estimate.
 *
 * The example waveform on UM10034P p.21 (32 chirps, 1270 us chirp period,
 * lambda = 12.4 mm) gives v_max = 2.45 m/s and dv = 0.15 m/s, so this replaces
 * the frame-to-frame range difference, which is quantised to whole bins.
 *
 * @param[in] iq_bytes Assembled frame IQ (chirp x RX x bin, int16 LE pairs).
 * @param[in] bin Range bin of the peak.
 * @param[in] range_bins / num_rx / chirp_number Frame geometry.
 * @param[in] chirp_period_s Time between chirps (frame period / chirp_number).
 * @param[in] wavelength_m Carrier wavelength (m), c / centre frequency.
 * @param[out] velocity_mps Radial velocity; positive = moving away.
 * @param[out] quality Coherent fraction in [0, 1]; optional.
 * @return True when at least three chirps and valid geometry were available.
 */
bool EstimateIcl1122DopplerVelocity(const std::vector<std::uint8_t>& iq_bytes,
                                    std::size_t bin, std::uint32_t range_bins,
                                    std::uint32_t num_rx,
                                    std::uint32_t chirp_number,
                                    double chirp_period_s,
                                    double wavelength_m,
                                    float* velocity_mps,
                                    float* quality = nullptr);

/**
 * @brief Emit every positive-magnitude range bin (for PointCloud spectrum).
 * @param[in] skip_near_bins Skip the first N bins (DC / TX leakage).
 */
void ExtractIcl1122RangeSpectrum(const std::vector<float>& bin_magnitude,
                                 double range_resolution_m,
                                 std::size_t skip_near_bins,
                                 std::vector<float>* xs, std::vector<float>* ys,
                                 std::vector<float>* intensities);

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_ICL1122_RANGE_FFT_HPP_
