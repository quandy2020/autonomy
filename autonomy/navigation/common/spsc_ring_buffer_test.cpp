/* Copyright 2026 The Navigation Authors (navigation) */
#include <atomic>
#include <iostream>
#include <thread>

#include "navigation/common/spsc_ring_buffer.hpp"

int main()
{
    using navigation::common::SpscRingBuffer;

    SpscRingBuffer<int, 4> queue;
    if (!queue.TryPush(1) || !queue.TryPush(2) || !queue.TryPush(3))
    {
        std::cerr << "push failed\n";
        return 1;
    }
    if (queue.TryPush(4))
    {
        std::cerr << "full queue accepted\n";
        return 1;
    }
    int value = 0;
    if (!queue.TryPop(&value) || value != 1 || queue.Size() != 2)
    {
        std::cerr << "pop failed\n";
        return 1;
    }
    if (!queue.TryPush(4))
    {
        std::cerr << "wrapped push failed\n";
        return 1;
    }
    if (queue.TryPush(5))
    {
        std::cerr << "wrapped full accepted\n";
        return 1;
    }
    if (!queue.TryPop(&value) || value != 2)
    {
        std::cerr << "wrap pop failed\n";
        return 1;
    }
    if (!queue.TryPush(5))
    {
        std::cerr << "second wrapped push failed\n";
        return 1;
    }

    SpscRingBuffer<int, 256> concurrent;
    std::atomic<bool> failed{false};
    std::thread producer([&concurrent]() {
        for (int i = 0; i < 10000; ++i)
        {
            while (!concurrent.TryPush(i))
            {
                std::this_thread::yield();
            }
        }
    });
    std::thread consumer([&concurrent, &failed]() {
        int expected = 0;
        while (expected < 10000)
        {
            int item = 0;
            if (!concurrent.TryPop(&item))
            {
                std::this_thread::yield();
                continue;
            }
            if (item != expected)
            {
                failed.store(true);
                return;
            }
            ++expected;
        }
    });
    producer.join();
    consumer.join();
    if (failed.load())
    {
        std::cerr << "spsc ordering failed\n";
        return 1;
    }
    return 0;
}
