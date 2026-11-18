/* Copyright 2026 The Openbot Authors. */
/** @file native_acquisition_stamp_test.cpp
 * @brief Check exact native epoch timestamps through the actual Atlas message adapter.
 * @details Links the production bridge and generated automsgs definitions. It
 * tests message identity and epoch invalidation, not visual tracking accuracy.
 */
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "atlas/message_adapter.hpp"

namespace autonomy::localization::atlas {

/** @brief Existing friend seam exercises production Record/GetOdometry/GetPath. */
class MessageAdapterEpochTest {
public:
    /** @brief Native epoch values that cannot all survive double-second reconstruction. */
    static void ExactAcquisition() {
        MessageAdapter adapter(nullptr);
        TrackingResult result;
        result.valid = true; result.map_id_valid = true; result.coordinate_frame_ready = true;
        result.map_id = 3; result.coordinate_epoch = 5;
        ::automsgs::msgs::builtin_interfaces::Time acquisition;
        acquisition.set_sec(1795004291); acquisition.set_nanosec(788033485);
        const auto nanoseconds = static_cast<std::int64_t>(acquisition.sec()) * 1000000000LL + acquisition.nanosec();
        result.time = static_cast<double>(nanoseconds) * 1e-9;
        Require(adapter.Record(result, &acquisition), "actual adapter recorded completed source acquisition");
        ::automsgs::msgs::nav_msgs::Odometry odometry;
        ::automsgs::msgs::nav_msgs::Path path;
        Require(adapter.GetOdometry(&odometry) && adapter.GetPath(&path), "real bridge produced native outputs");
        Require(Same(odometry.header().stamp(), acquisition) &&
            Same(odometry.pose().pose().header().stamp(), acquisition) &&
            Same(path.header().stamp(), acquisition) && Same(path.poses(0).header().stamp(), acquisition),
            "all pose/path Header identities retain exact input epoch nanoseconds");

        acquisition.set_nanosec(788033486);
        Require(adapter.Record(result, &acquisition), "next distinct integer acquisition recorded");
        Require(adapter.GetPath(&path) && path.poses_size() == 2 &&
            path.poses(0).header().stamp().nanosec() == 788033485 &&
            path.poses(1).header().stamp().nanosec() == 788033486,
            "two source stamps remain distinct even when their double times coincide");

        result.valid = false; ++result.coordinate_epoch;
        Require(!adapter.Record(result, &acquisition) && !adapter.GetOdometry(&odometry) && !adapter.GetPath(&path),
            "invalid coordinate boundary cannot reuse the previous valid pose or path");
        result.valid = true; acquisition.set_sec(1795004292); acquisition.set_nanosec(999999999);
        result.time = static_cast<double>(acquisition.sec()) + acquisition.nanosec() * 1e-9;
        Require(adapter.Record(result, &acquisition) && adapter.GetOdometry(&odometry) && adapter.GetPath(&path) &&
            path.poses_size() == 1 && Same(odometry.header().stamp(), acquisition),
            "near-second-boundary acquisition remains exact after an actual coordinate rebase");
    }
private:
    /** @brief Fail with the specific native message-identity invariant. */
    static void Require(bool condition, const char* reason) {
        if (!condition) throw std::runtime_error(reason);
    }
    /** @brief Compare native integer fields without floating-point tolerance. */
    static bool Same(const ::automsgs::msgs::builtin_interfaces::Time& first,
                     const ::automsgs::msgs::builtin_interfaces::Time& second) {
        return first.sec() == second.sec() && first.nanosec() == second.nanosec();
    }
};
}  // namespace autonomy::localization::atlas

/** @brief Run the actual native bridge regression without constructing a mock tracker. */
int main() {
    try {
        autonomy::localization::atlas::MessageAdapterEpochTest::ExactAcquisition();
        std::cout << "PASS exact_acquisition_stamp" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL exact_acquisition_stamp: " << error.what() << std::endl;
        return 1;
    }
}
