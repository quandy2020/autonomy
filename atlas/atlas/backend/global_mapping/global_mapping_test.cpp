/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/global_mapping.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

TEST(GlobalMapping, GeneratesIncreasingRequestIdentifiers) {
  GlobalMapping global_mapping;
  EXPECT_EQ(global_mapping.NextRequestId(), 1u);
  EXPECT_EQ(global_mapping.NextRequestId(), 2u);
}

TEST(GlobalMapping, GeneratesUniqueRequestIdentifiersConcurrently) {
  constexpr std::size_t worker_count = 4;
  constexpr std::size_t identifiers_per_worker = 100;
  std::vector<std::uint64_t> identifiers(worker_count * identifiers_per_worker);
  GlobalMapping global_mapping;
  std::vector<std::thread> workers;
  workers.reserve(worker_count);
  for (std::size_t worker_index = 0; worker_index < worker_count;
       ++worker_index) {
    workers.emplace_back([worker_index, &identifiers, &global_mapping] {
      const std::size_t first_identifier =
          worker_index * identifiers_per_worker;
      for (std::size_t index = 0; index < identifiers_per_worker; ++index) {
        identifiers[first_identifier + index] = global_mapping.NextRequestId();
      }
    });
  }
  for (std::thread& worker : workers) worker.join();

  std::sort(identifiers.begin(), identifiers.end());
  for (std::size_t index = 0; index < identifiers.size(); ++index) {
    EXPECT_EQ(identifiers[index], index + 1);
  }
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
