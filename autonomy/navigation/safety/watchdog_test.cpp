/* Copyright 2026 The Navigation Authors (navigation) */
#include <chrono>
#include <iostream>
#include <thread>

#include "navigation/safety/watchdog.hpp"

int main()
{
    navigation::safety::Watchdog short_budget(2);
    if (short_budget.Expired())
    {
        std::cerr << "expired immediately\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!short_budget.Expired())
    {
        std::cerr << "deadline not detected\n";
        return 1;
    }
    navigation::safety::Watchdog long_budget(5000);
    if (long_budget.Expired())
    {
        std::cerr << "long budget expired\n";
        return 1;
    }
    return 0;
}
