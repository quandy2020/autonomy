/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/guardian/fault_classifier.hpp"

int main()
{
    using navigation::guardian::FaultClassifier;
    using navigation::lifecycle::DiagnosticReport;

    DiagnosticReport info;
    info.set_code(1001);
    info.set_level(navigation::lifecycle::DIAGNOSTIC_LEVEL_INFO);
    if (FaultClassifier::Classify(info, 1) != navigation::lifecycle::FAULT_CLASS_TRANSIENT)
    {
        std::cerr << "transient failed\n";
        return 1;
    }
    if (FaultClassifier::Classify(info, 3) != navigation::lifecycle::FAULT_CLASS_PERSISTENT)
    {
        std::cerr << "repeated transient failed\n";
        return 1;
    }
    DiagnosticReport fatal;
    fatal.set_code(6001);
    fatal.set_level(navigation::lifecycle::DIAGNOSTIC_LEVEL_ERROR);
    if (FaultClassifier::Classify(fatal, 1) != navigation::lifecycle::FAULT_CLASS_FATAL)
    {
        std::cerr << "fatal failed\n";
        return 1;
    }
    return 0;
}
