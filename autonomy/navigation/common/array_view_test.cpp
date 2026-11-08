/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/common/array_view.hpp"

int main()
{
    using navigation::common::ArrayView;

    int values[4] = {1, 2, 3, 4};
    const ArrayView<int> view(values);
    if (view.size() != 4 || view.empty() || view[2] != 3)
    {
        std::cerr << "view failed\n";
        return 1;
    }
    int sum = 0;
    for (int value : view)
    {
        sum += value;
    }
    if (sum != 10)
    {
        std::cerr << "iteration failed\n";
        return 1;
    }
    const ArrayView<int> sub = view.Subview(1, 2);
    if (sub.size() != 2 || sub[0] != 2 || sub[1] != 3)
    {
        std::cerr << "subview failed\n";
        return 1;
    }
    return 0;
}
