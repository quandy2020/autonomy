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

#include "autonomy/audio/inference/direction_detection.hpp"

#include "autonomy/audio/inference/fft.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/math_utils.hpp"

#include "autolink/common/file.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <complex>

namespace autonomy {
namespace audio {

std::pair<automsgs::msgs::geometry_msgs::Point, double>
DirectionDetection::EstimateSoundSource(
    std::vector<std::vector<double>>&& channels_vec,
    const std::string& respeaker_extrinsic_file, const int sample_rate,
    const double mic_distance) {
    if (!respeaker2imu_ptr_) {
        respeaker2imu_ptr_ = std::make_unique<Eigen::Matrix4d>();
        if (respeaker_extrinsic_file.empty() ||
            !LoadExtrinsics(respeaker_extrinsic_file,
                            respeaker2imu_ptr_.get())) {
            respeaker2imu_ptr_->setIdentity();
        }
    }
    const double degree =
        EstimateDirection(std::move(channels_vec), sample_rate, mic_distance);
    Eigen::Vector4d source_position(kDistance * std::sin(degree),
                                    kDistance * std::cos(degree), 0.0, 1.0);
    source_position = (*respeaker2imu_ptr_) * source_position;

    automsgs::msgs::geometry_msgs::Point source_position_p3d;
    source_position_p3d.set_x(source_position[0]);
    source_position_p3d.set_y(source_position[1]);
    source_position_p3d.set_z(source_position[2]);
    return {source_position_p3d, common::math::NormalizeAngle(degree)};
}

double DirectionDetection::EstimateDirection(
    std::vector<std::vector<double>>&& channels_vec, const int sample_rate,
    const double mic_distance) {
    if (channels_vec.size() < 4 || channels_vec[0].empty()) {
        AERROR << "DirectionDetection needs 4 RAW channels.";
        return 0.0;
    }
    const double max_tau = mic_distance / kSoundSpeed;
    const double tau0 =
        GccPhat(channels_vec[0], channels_vec[2], sample_rate, max_tau, 1);
    const double theta0 = std::asin(std::clamp(tau0 / max_tau, -1.0, 1.0)) *
                          180.0 / M_PI;
    const double tau1 =
        GccPhat(channels_vec[1], channels_vec[3], sample_rate, max_tau, 1);
    const double theta1 = std::asin(std::clamp(tau1 / max_tau, -1.0, 1.0)) *
                          180.0 / M_PI;

    int best_guess = 0;
    if (std::fabs(theta0) < std::fabs(theta1)) {
        best_guess = theta1 > 0 ? static_cast<int>(std::fmod(theta0 + 360.0, 360.0))
                               : static_cast<int>(180.0 - theta0);
    } else {
        best_guess = theta0 < 0 ? static_cast<int>(std::fmod(theta1 + 360.0, 360.0))
                               : static_cast<int>(180.0 - theta1);
        best_guess = (best_guess + 90 + 180) % 360;
    }
    best_guess = (-best_guess + 480) % 360;
    return static_cast<double>(best_guess) / 180.0 * M_PI;
}

bool DirectionDetection::LoadExtrinsics(const std::string& yaml_file,
                                        Eigen::Matrix4d* respeaker_extrinsic) {
    if (!autolink::common::PathExists(yaml_file)) {
        AINFO << yaml_file << " does not exist; using identity extrinsics.";
        return false;
    }
    try {
        const YAML::Node node = YAML::LoadFile(yaml_file);
        if (node.IsNull()) {
            AERROR << "Load " << yaml_file << " failed.";
            return false;
        }
        const double qw = node["transform"]["rotation"]["w"].as<double>();
        const double qx = node["transform"]["rotation"]["x"].as<double>();
        const double qy = node["transform"]["rotation"]["y"].as<double>();
        const double qz = node["transform"]["rotation"]["z"].as<double>();
        const double tx = node["transform"]["translation"]["x"].as<double>();
        const double ty = node["transform"]["translation"]["y"].as<double>();
        const double tz = node["transform"]["translation"]["z"].as<double>();

        respeaker_extrinsic->setZero();
        Eigen::Quaterniond q(qw, qx, qy, qz);
        respeaker_extrinsic->block<3, 3>(0, 0) =
            q.normalized().toRotationMatrix();
        (*respeaker_extrinsic)(0, 3) = tx;
        (*respeaker_extrinsic)(1, 3) = ty;
        (*respeaker_extrinsic)(2, 3) = tz;
        (*respeaker_extrinsic)(3, 3) = 1.0;
        return true;
    } catch (const YAML::Exception& e) {
        AERROR << "Load extrinsics " << yaml_file << " failed: " << e.what();
        return false;
    }
}

double DirectionDetection::GccPhat(const std::vector<double>& sig,
                                   const std::vector<double>& refsig,
                                   const int fs, const double max_tau,
                                   const int interp) {
    const int n_sig = static_cast<int>(sig.size());
    const int n_refsig = static_cast<int>(refsig.size());
    const int n = n_sig + n_refsig;
    if (n_sig == 0 || n_refsig == 0 || fs <= 0) {
        return 0.0;
    }

    std::vector<double> psig(n, 0.0);
    std::vector<double> prefsig(n, 0.0);
    std::copy(sig.begin(), sig.end(), psig.begin());
    std::copy(refsig.begin(), refsig.end(), prefsig.begin());

    auto Spec = [](const std::vector<double>& x) {
        return Rfft(x);
    };
    auto psig_f = Spec(psig);
    auto prefsig_f = Spec(prefsig);
    const int bins = static_cast<int>(
        std::min(psig_f.size(), prefsig_f.size()));

    std::vector<std::complex<double>> r(bins);
    for (int i = 0; i < bins; ++i) {
        const std::complex<double> cross = psig_f[i] * std::conj(prefsig_f[i]);
        const double mag = std::abs(cross);
        r[i] = mag > 1e-12 ? cross / mag : std::complex<double>(0.0, 0.0);
    }

    // irfft length: use padded FFT size matching Rfft of length-n input.
    const int n_fft = NextPowerOfTwo(n);
    std::vector<double> cc = Irfft(r, n_fft);
    if (static_cast<int>(cc.size()) < n_fft) {
        return 0.0;
    }

    int max_shift = interp * n_fft / 2;
    if (max_tau != 0.0) {
        max_shift = std::min(static_cast<int>(interp * fs * max_tau), max_shift);
    }
    max_shift = std::max(0, max_shift);

    std::vector<double> window;
    window.reserve(2 * max_shift + 1);
    for (int i = n_fft - max_shift; i < n_fft; ++i) {
        window.push_back(cc[i]);
    }
    for (int i = 0; i <= max_shift; ++i) {
        window.push_back(cc[i]);
    }

    int peak = 0;
    double peak_abs = -1.0;
    for (int i = 0; i < static_cast<int>(window.size()); ++i) {
        const double a = std::fabs(window[i]);
        if (a > peak_abs) {
            peak_abs = a;
            peak = i;
        }
    }
    const int shift = peak - max_shift;
    return shift / static_cast<double>(interp * fs);
}

}  // namespace audio
}  // namespace autonomy
