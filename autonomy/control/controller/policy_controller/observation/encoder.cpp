/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/encoder.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "autonomy/control/controller/policy_controller/observation/costmap_encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/image_encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/kinematic_encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/laser_scan_encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/path_encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/point_cloud_encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/tensor_frame_encoder.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

bool ApplyNormalization(const proto::TensorNorm& statistics,
                        std::vector<float>* data) {
    if (data == nullptr ||
        (statistics.mean_size() == 0 && statistics.stddev_size() == 0)) {
        return true;
    }
    for (std::size_t i = 0; i < data->size(); ++i) {
        float mean = 0.0f;
        float stddev = 1.0f;
        if (statistics.mean_size() > 0) {
            mean = statistics.mean(static_cast<int>(
                std::min<std::size_t>(i, statistics.mean_size() - 1)));
        }
        if (statistics.stddev_size() > 0) {
            stddev = statistics.stddev(static_cast<int>(
                std::min<std::size_t>(i, statistics.stddev_size() - 1)));
            if (std::fabs(stddev) < 1e-8f) {
                stddev = 1.0f;
            }
        }
        (*data)[i] = ((*data)[i] - mean) / stddev;
    }
    return true;
}

double ModalityStamp(const ObservationSnapshot& snapshot,
                     proto::ObservationModality modality,
                     const std::string& raw_name) {
    switch (modality) {
        case proto::OBSERVATION_KINEMATIC:
            return snapshot.pose.has_value()
                       ? HeaderStampSeconds(snapshot.pose->header())
                       : 0.0;
        case proto::OBSERVATION_LASER:
            return snapshot.laser_scan != nullptr
                       ? HeaderStampSeconds(snapshot.laser_scan->header())
                       : 0.0;
        case proto::OBSERVATION_IMAGE:
            return snapshot.image != nullptr
                       ? HeaderStampSeconds(snapshot.image->header())
                       : 0.0;
        case proto::OBSERVATION_POINT_CLOUD:
            return snapshot.point_cloud != nullptr
                       ? HeaderStampSeconds(snapshot.point_cloud->header())
                       : 0.0;
        case proto::OBSERVATION_LIDAR_BEV:
            return snapshot.lidar_bev != nullptr ? snapshot.lidar_bev->stamp_sec
                                                 : 0.0;
        case proto::OBSERVATION_RAW: {
            const auto it = snapshot.raw_tensors.find(raw_name);
            return it != snapshot.raw_tensors.end() && it->second != nullptr
                       ? it->second->stamp_sec
                       : 0.0;
        }
        default:
            return 0.0;
    }
}

std::size_t ExpectedElements(const proto::ObservationInput& input,
                             const ObservationEncodeContext& context) {
    std::size_t count = 1;
    bool any = false;
    for (int i = 0; i < input.shape_size(); ++i) {
        if (input.shape(i) > 0) {
            count *= static_cast<std::size_t>(input.shape(i));
            any = true;
        }
    }
    if (any) {
        return count;
    }
    switch (input.modality()) {
        case proto::OBSERVATION_KINEMATIC:
            return 6;
        case proto::OBSERVATION_PATH:
            return static_cast<std::size_t>(
                std::max(1, context.path_point_count) * 2);
        case proto::OBSERVATION_COSTMAP:
            return 64 * 64;
        case proto::OBSERVATION_LASER:
            return static_cast<std::size_t>(
                std::max(1, context.laser_scan_bin_count));
        case proto::OBSERVATION_IMAGE:
            return 3 * 224 * 224;
        case proto::OBSERVATION_POINT_CLOUD:
            return static_cast<std::size_t>(
                std::max(1, context.point_cloud_points) * 3);
        case proto::OBSERVATION_LIDAR_BEV:
            return 200 * 200;
        case proto::OBSERVATION_RAW:
            return 1;
        default:
            return 1;
    }
}

const char* DefaultModalityName(proto::ObservationModality modality) {
    switch (modality) {
        case proto::OBSERVATION_KINEMATIC:
            return "observation_kinematic";
        case proto::OBSERVATION_PATH:
            return "observation_path";
        case proto::OBSERVATION_COSTMAP:
            return "observation_costmap";
        case proto::OBSERVATION_LASER:
            return "observation_laser";
        case proto::OBSERVATION_IMAGE:
            return "observation_image";
        case proto::OBSERVATION_POINT_CLOUD:
            return "observation_point_cloud";
        case proto::OBSERVATION_LIDAR_BEV:
            return "observation_lidar_bev";
        case proto::OBSERVATION_RAW:
            return "observation_raw";
        default:
            return "";
    }
}

double LastGoodWallSec(const ObservationSnapshot& snapshot,
                       const proto::ObservationInput& input) {
    const auto it = snapshot.modality_last_wall_sec.find(input.name());
    if (it != snapshot.modality_last_wall_sec.end() && it->second > 0.0) {
        return it->second;
    }
    const char* fallback = DefaultModalityName(input.modality());
    if (fallback[0] == '\0' || input.name() == fallback) {
        return 0.0;
    }
    const auto jt = snapshot.modality_last_wall_sec.find(fallback);
    if (jt != snapshot.modality_last_wall_sec.end()) {
        return jt->second;
    }
    return 0.0;
}

bool InMissingGrace(const ObservationSnapshot& snapshot,
                    const proto::ObservationInput& input,
                    const ObservationEncodeContext& context) {
    if (context.missing_modality_grace_sec <= 0.0 || context.now_sec <= 0.0) {
        return false;
    }
    const double last = LastGoodWallSec(snapshot, input);
    if (last <= 0.0) {
        return false;
    }
    return (context.now_sec - last) <= context.missing_modality_grace_sec;
}

bool PassesTimeChecks(const ObservationSnapshot& snapshot,
                      const proto::ObservationInput& input,
                      const ObservationEncodeContext& context,
                      std::string* error) {
    const double modality_stamp =
        ModalityStamp(snapshot, input.modality(), input.name());
    if (modality_stamp <= 0.0) {
        return true;
    }

    const double pose_stamp =
        snapshot.pose.has_value() ? HeaderStampSeconds(snapshot.pose->header())
                                  : snapshot.stamp_sec;

    if (context.max_observation_skew_sec > 0.0 && pose_stamp > 0.0) {
        const double skew = std::fabs(modality_stamp - pose_stamp);
        if (skew > context.max_observation_skew_sec) {
            if (error != nullptr) {
                *error = "Observation skew too large for " + input.name();
            }
            return false;
        }
    }

    if (context.max_observation_age_sec > 0.0 && snapshot.stamp_sec > 0.0) {
        const double age = std::fabs(snapshot.stamp_sec - modality_stamp);
        if (age > context.max_observation_age_sec &&
            (input.modality() == proto::OBSERVATION_LASER ||
             input.modality() == proto::OBSERVATION_IMAGE ||
             input.modality() == proto::OBSERVATION_POINT_CLOUD ||
             input.modality() == proto::OBSERVATION_LIDAR_BEV ||
             input.modality() == proto::OBSERVATION_RAW)) {
            if (error != nullptr) {
                *error = "Stale observation for " + input.name();
            }
            return false;
        }
    }
    return true;
}

bool EncodeOne(const ObservationSnapshot& snapshot,
               const proto::ObservationInput& input,
               const ObservationEncodeContext& context,
               std::vector<float>* output, std::string* error) {
    bool ok = false;
    switch (input.modality()) {
        case proto::OBSERVATION_KINEMATIC:
            ok = EncodeKinematic(snapshot, input, output, error,
                                 context.relative_kinematic);
            break;
        case proto::OBSERVATION_PATH:
            ok = EncodePath(snapshot, input, context.path_point_count, output,
                            error);
            break;
        case proto::OBSERVATION_COSTMAP:
            ok = EncodeCostmap(snapshot, input, output, error);
            break;
        case proto::OBSERVATION_LASER:
            ok = EncodeLaserScan(snapshot, input, context.laser_scan_bin_count,
                                 output, error);
            break;
        case proto::OBSERVATION_IMAGE:
            ok = EncodeImage(snapshot, input, output, error);
            break;
        case proto::OBSERVATION_POINT_CLOUD:
            ok = EncodePointCloud(snapshot, input, context.point_cloud_points,
                                  output, error,
                                  context.point_cloud_stride_sample);
            break;
        case proto::OBSERVATION_LIDAR_BEV:
            ok = EncodeLidarBev(snapshot, input, output, error);
            break;
        case proto::OBSERVATION_RAW:
            ok = EncodeRawTensor(snapshot, input, output, error);
            break;
        default:
            if (error != nullptr) {
                *error = "Unknown modality for " + input.name();
            }
            return false;
    }
    if (!ok) {
        return false;
    }
    return ApplyNormalization(input.norm(), output);
}

bool HandleRequiredUnavailable(const ObservationSnapshot& snapshot,
                               const proto::ObservationInput& input,
                               const ObservationEncodeContext& context,
                               ::autonomy::common::network::FloatTensorMap* inputs,
                               bool* zero_filled, std::string* error) {
    const bool in_grace = InMissingGrace(snapshot, input, context);
    if (!in_grace ||
        context.missing_modality_policy == proto::MISSING_MODALITY_FAIL) {
        if (error != nullptr) {
            *error = "Missing required modality for " + input.name();
        }
        return false;
    }
    if (context.missing_modality_policy == proto::MISSING_MODALITY_HOLD) {
        if (error != nullptr) {
            *error = "Holding on missing modality for " + input.name();
        }
        return false;
    }
    // ZERO_FILL within grace.
    (*inputs)[input.name()] = std::vector<float>(
        ExpectedElements(input, context), 0.0f);
    if (zero_filled != nullptr) {
        *zero_filled = true;
    }
    return true;
}

}  // namespace

bool EncodeObservations(const ObservationSnapshot& snapshot,
                        const PolicyConfig& config,
                        const ObservationEncodeContext& context,
                        ::autonomy::common::network::FloatTensorMap* inputs,
                        std::string* error, bool* zero_filled) {
    if (inputs == nullptr) {
        return false;
    }
    inputs->clear();
    if (zero_filled != nullptr) {
        *zero_filled = false;
    }

    for (const auto& input : config.proto().inputs()) {
        if (input.name().empty()) {
            continue;
        }
        if (!HasObservationModality(snapshot, input.modality(), input.name())) {
            if (input.required()) {
                if (!HandleRequiredUnavailable(snapshot, input, context, inputs,
                                               zero_filled, error)) {
                    return false;
                }
            }
            continue;
        }

        std::string time_error;
        if (!PassesTimeChecks(snapshot, input, context, &time_error)) {
            if (!input.required()) {
                continue;
            }
            if (HandleRequiredUnavailable(snapshot, input, context, inputs,
                                          zero_filled, error)) {
                continue;  // ZERO_FILL within grace
            }
            if (error != nullptr) {
                *error = time_error;
            }
            return false;
        }

        std::vector<float> tensor;
        std::string local_error;
        if (!EncodeOne(snapshot, input, context, &tensor, &local_error)) {
            if (input.required()) {
                if (error != nullptr) {
                    *error = local_error;
                }
                return false;
            }
            continue;
        }
        (*inputs)[input.name()] = std::move(tensor);
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
