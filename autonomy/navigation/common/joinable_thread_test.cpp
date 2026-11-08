/* Copyright 2026 The Navigation Authors (navigation) */
#include <atomic>
#include <iostream>

#include "navigation/common/joinable_thread.hpp"

int main()
{
    using navigation::common::JoinableThread;

    std::atomic<int> counter{0};
    {
        JoinableThread thread([&counter]() { counter.fetch_add(1); });
        if (!thread.Joinable())
        {
            std::cerr << "thread not joinable\n";
            return 1;
        }
    }
    if (counter.load() != 1)
    {
        std::cerr << "thread did not run\n";
        return 1;
    }
    return 0;
}
