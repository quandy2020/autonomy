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

#include "atlas/instance_lock.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

class TemporaryLockPath
{
public:
    TemporaryLockPath() {
        static std::atomic<unsigned int> sequence{0};
        path_ = std::filesystem::temp_directory_path() /
                ("atlas_instance_lock_test_" +
                 std::to_string(::getpid()) + "_" +
                 std::to_string(sequence.fetch_add(1)) + ".lock");
    }

    ~TemporaryLockPath() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    [[nodiscard]] const std::filesystem::path& path() const {
        return path_;
    }

private:
    std::filesystem::path path_;
};

TEST(InstanceLockTest, RejectsSecondOwnerAndReportsMetadata) {
    TemporaryLockPath path;
    InstanceLock first;
    InstanceLock second;
    std::string error;

    ASSERT_TRUE(first.TryAcquire(path.path().string(), "first.yaml", &error))
        << error;
    EXPECT_FALSE(second.TryAcquire(path.path().string(), "second.yaml", &error));
    EXPECT_NE(error.find("another Atlas instance is already running"),
              std::string::npos);
    EXPECT_NE(error.find("pid=" + std::to_string(::getpid())),
              std::string::npos);
    EXPECT_NE(error.find("config=first.yaml"), std::string::npos);
}

TEST(InstanceLockTest, ReleaseAllowsNextOwner) {
    TemporaryLockPath path;
    InstanceLock first;
    InstanceLock second;
    std::string error;

    ASSERT_TRUE(first.TryAcquire(path.path().string(), "first.yaml", &error))
        << error;
    first.Release();
    ASSERT_TRUE(second.TryAcquire(path.path().string(), "second.yaml", &error))
        << error;

    std::ifstream metadata(path.path());
    std::string line;
    std::getline(metadata, line);
    EXPECT_NE(line.find("config=second.yaml"), std::string::npos);
}

TEST(InstanceLockTest, DestructorReleasesLock) {
    TemporaryLockPath path;
    std::string error;
    {
        InstanceLock first;
        ASSERT_TRUE(first.TryAcquire(path.path().string(), "first.yaml", &error))
            << error;
    }

    InstanceLock second;
    EXPECT_TRUE(second.TryAcquire(path.path().string(), "second.yaml", &error))
        << error;
}

}  // namespace
}  // namespace autonomy::localization::atlas
