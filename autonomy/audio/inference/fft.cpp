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

#include "autonomy/audio/inference/fft.hpp"

#include <cmath>

namespace autonomy {
namespace audio {

int NextPowerOfTwo(int n) {
    int p = 1;
    while (p < n) {
        p <<= 1;
    }
    return p;
}

void Fft(std::vector<std::complex<double>>* data, bool inverse) {
    const int n = static_cast<int>(data->size());
    if (n <= 1) {
        return;
    }
    // Bit-reversal permutation.
    for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap((*data)[i], (*data)[j]);
        }
    }
    const double sign = inverse ? 1.0 : -1.0;
    for (int len = 2; len <= n; len <<= 1) {
        const double ang = sign * 2.0 * M_PI / static_cast<double>(len);
        const std::complex<double> wlen(std::cos(ang), std::sin(ang));
        for (int i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (int j = 0; j < len / 2; ++j) {
                const std::complex<double> u = (*data)[i + j];
                const std::complex<double> v = (*data)[i + j + len / 2] * w;
                (*data)[i + j] = u + v;
                (*data)[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
    if (inverse) {
        for (auto& sample : *data) {
            sample /= static_cast<double>(n);
        }
    }
}

std::vector<std::complex<double>> Rfft(const std::vector<double>& real) {
    const int n = NextPowerOfTwo(static_cast<int>(real.size()));
    std::vector<std::complex<double>> data(n, {0.0, 0.0});
    for (std::size_t i = 0; i < real.size(); ++i) {
        data[i] = {real[i], 0.0};
    }
    Fft(&data, false);
    data.resize(n / 2 + 1);
    return data;
}

std::vector<double> Irfft(const std::vector<std::complex<double>>& spectrum,
                          int n) {
    const int N = NextPowerOfTwo(n);
    std::vector<std::complex<double>> data(N, {0.0, 0.0});
    const int half = N / 2;
    for (int i = 0; i <= half && i < static_cast<int>(spectrum.size()); ++i) {
        data[i] = spectrum[i];
    }
    for (int i = 1; i < half; ++i) {
        data[N - i] = std::conj(data[i]);
    }
    Fft(&data, true);
    std::vector<double> out(N);
    for (int i = 0; i < N; ++i) {
        out[i] = data[i].real();
    }
    return out;
}

std::vector<std::complex<double>> Fft1d(const std::vector<double>& signal) {
    const int n = static_cast<int>(signal.size());
    if (n == 0) {
        return {};
    }
    const int N = NextPowerOfTwo(n);
    std::vector<std::complex<double>> data(N, {0.0, 0.0});
    for (int i = 0; i < n; ++i) {
        data[i] = {signal[i], 0.0};
    }
    Fft(&data, false);
    // Match Apollo fftw length-n output: keep first n bins after padding FFT.
    if (N != n) {
        data.resize(n);
    }
    return data;
}

}  // namespace audio
}  // namespace autonomy
