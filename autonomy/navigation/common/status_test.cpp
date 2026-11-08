/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/common/status.hpp"

int main()
{
    using navigation::common::ErrorCode;
    using navigation::common::Status;

    const Status ok;
    if (!ok.Ok() || ok.code != ErrorCode::kOk)
    {
        std::cerr << "default status must be ok\n";
        return 1;
    }
    const Status invalid = Status::InvalidArgument("bad");
    if (invalid.Ok() || invalid.code != ErrorCode::kInvalidArgument ||
        invalid.message != "bad")
    {
        std::cerr << "invalid status failed\n";
        return 1;
    }
    if (Status::Configuration("x").code != ErrorCode::kConfigurationError)
    {
        std::cerr << "configuration factory failed\n";
        return 1;
    }
    return 0;
}
