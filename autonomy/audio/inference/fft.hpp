/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_AUDIO_INFERENCE_FFT_HPP_
#define AUTONOMY_AUDIO_INFERENCE_FFT_HPP_

#include <complex>
#include <vector>

namespace autonomy {
namespace audio {

// In-place radix-2 Cooley–Tukey FFT. Size must be a power of two.
void Fft(std::vector<std::complex<double>>* data, bool inverse);

// Next power of two >= n (n > 0).
int NextPowerOfTwo(int n);

// Real FFT: pads to power of two, returns length N/2+1 spectrum of padded size N.
std::vector<std::complex<double>> Rfft(const std::vector<double>& real);

// Inverse of Rfft for a spectrum produced by Rfft of length n (padded).
std::vector<double> Irfft(const std::vector<std::complex<double>>& spectrum,
                          int n);

// Full-length complex DFT via power-of-two FFT with zero padding.
std::vector<std::complex<double>> Fft1d(const std::vector<double>& signal);

}  // namespace audio
}  // namespace autonomy

#endif  // AUTONOMY_AUDIO_INFERENCE_FFT_HPP_
