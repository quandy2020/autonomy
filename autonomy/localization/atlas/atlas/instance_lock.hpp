/*
 * Copyright 2026 The Openbot Authors
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

#pragma once

#include <string>

namespace autonomy::localization::atlas {

/** Holds the process-wide lock that protects Atlas output topics. */
class InstanceLock
{
public:
    InstanceLock() = default;
    ~InstanceLock();

    InstanceLock(const InstanceLock&) = delete;
    InstanceLock& operator=(const InstanceLock&) = delete;

    /**
     * Acquires path without waiting and records owner metadata.
     *
     * Returns false when another Atlas process owns the lock or the lock file
     * cannot be used. error_message receives a user-facing diagnostic.
     */
    bool TryAcquire(const std::string& path, const std::string& config_path,
                    std::string* error_message);

    /** Releases the held lock. Safe to call more than once. */
    void Release();

    [[nodiscard]] bool held() const {
        return descriptor_ >= 0;
    }

private:
    int descriptor_{-1};
};

/** Returns the absolute Atlas lock path, with a test/domain override. */
std::string ResolveInstanceLockPath();

}  // namespace autonomy::localization::atlas
