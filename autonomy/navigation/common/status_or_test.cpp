/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <string>

#include "navigation/common/status_or.hpp"

int main()
{
    using navigation::common::Status;
    using navigation::common::StatusOr;

    StatusOr<int> value(42);
    if (!value.Ok() || value.Value() != 42 || value.ValueOr(0) != 42)
    {
        std::cerr << "value case failed\n";
        return 1;
    }
    StatusOr<int> error(Status::Unavailable("down"));
    if (error.Ok() || error.ValueOr(7) != 7 ||
        error.Error().code != navigation::common::ErrorCode::kUnavailable)
    {
        std::cerr << "error case failed\n";
        return 1;
    }
    StatusOr<std::string> text(std::string("hello"));
    if (text.Value() != "hello")
    {
        std::cerr << "move case failed\n";
        return 1;
    }
    return 0;
}
