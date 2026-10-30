/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/system.hpp"

#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::internal {
namespace {

TEST(MapChangeObserverTest, DetectsRevisionAdvanceAndRollback) {
    MapChangeObserver observer;
    EXPECT_FALSE(observer.Observe(7, 0));
    EXPECT_FALSE(observer.Observe(7, 0));
    EXPECT_TRUE(observer.Observe(7, 1));
    EXPECT_FALSE(observer.Observe(7, 1));
    EXPECT_TRUE(observer.Observe(7, 0));
    EXPECT_FALSE(observer.Observe(7, 0));
}

TEST(MapChangeObserverTest, DetectsMapReplacementAtSameRevision) {
    MapChangeObserver observer;
    EXPECT_FALSE(observer.Observe(7, 0));
    EXPECT_TRUE(observer.Observe(8, 0));
    EXPECT_FALSE(observer.Observe(8, 0));
}

TEST(MapChangeObserverTest, DoesNotShareStateAcrossSystems) {
    MapChangeObserver first;
    MapChangeObserver second;
    EXPECT_FALSE(first.Observe(1, 0));
    EXPECT_TRUE(first.Observe(1, 1));
    EXPECT_FALSE(second.Observe(1, 0));
    EXPECT_TRUE(second.Observe(1, 1));
}

TEST(MapChangeObserverTest, ReportsOneChangeToConcurrentCallers) {
    MapChangeObserver observer;
    ASSERT_FALSE(observer.Observe(1, 0));
    std::atomic<int> changes{0};
    std::vector<std::thread> callers;
    for (int index = 0; index < 8; ++index) {
        callers.emplace_back([&observer, &changes] {
            if (observer.Observe(1, 1))
                ++changes;
        });
    }
    for (std::thread& caller : callers)
        caller.join();
    EXPECT_EQ(changes.load(), 1);
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
