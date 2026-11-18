/* Copyright 2026 The Openbot Authors. */
/** @file motion_admission.hpp @brief Scoped scheduler admission with a retained movement permit. */
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <utility>

namespace autonomy::task::common {

/**
 * @brief One pending admission; completion never renews its movement permission.
 * @details This is an in-process ownership handle, not a transport message.
 * The domain retains PermitQuery() in its execution gate and completes this
 * handle after committing or rejecting admission. Destruction reports rejection
 * when admission unwinds through an exception. The scheduler callback must not
 * throw; no task lifecycle mutex may be acquired on the accepted path.
 */
class MotionAdmission {
public:
    using Permit = std::function<bool()>;  ///< Bounded, nonblocking movement eligibility query.
    using Completion = std::function<void(bool)>;  ///< Exactly one admission outcome.
    using Factory = std::function<std::shared_ptr<MotionAdmission>()>;  ///< Called outside domain locks.

    /** @brief Capture immutable eligibility and one completion callback. */
    MotionAdmission(Permit permit, Completion completion)
        : permit_(std::move(permit)), completion_(std::move(completion)) {}
    /** @brief An uncommitted admission is rejected, including on exception. */
    ~MotionAdmission() { Complete(false); }
    MotionAdmission(const MotionAdmission&) = delete;
    MotionAdmission& operator=(const MotionAdmission&) = delete;

    /** @return Eligibility retained independently of this pending-admission handle. */
    Permit PermitQuery() const { return permit_; }
    /** @return False for missing, denied or exceptional eligibility. */
    bool Valid() const noexcept {
        try { return permit_ && permit_(); } catch (...) { return false; }
    }
    /** @brief Publish the outcome once; duplicate completion is harmless. */
    void Complete(bool accepted) noexcept {
        if (finished_.exchange(true)) return;
        try { if (completion_) completion_(accepted); } catch (...) {}
    }

private:
    const Permit permit_;  ///< Execution eligibility does not change at acceptance.
    const Completion completion_;  ///< Owns only weak scheduler/task dependencies.
    std::atomic<bool> finished_{false};  ///< Single completion across shared handles.
};
}  // namespace autonomy::task::common
