"""
Autonomy Bazel Configuration

This file provides common configuration constants and utilities
for the autonomy project.
"""

# Common compiler options
AUTONOMY_COPTS = [
    "-std=c++17",
]

# Common exclude patterns
AUTONOMY_DEFAULT_EXCLUDES = [
    "**/*_test.hpp",
    "**/*_test.h",
    "**/*_test.cpp",
    "**/*_test.cc",
    "**/test/**",
    "**/testing/**",
]

# Common header patterns
AUTONOMY_HEADER_PATTERNS = [
    "*.hpp",
    "*.h",
    "**/*.hpp",
    "**/*.h",
]

# Common source patterns
AUTONOMY_SOURCE_PATTERNS = [
    "*.cpp",
    "*.cc",
    "**/*.cpp",
    "**/*.cc",
]

# Base dependencies that most modules need
AUTONOMY_BASE_DEPS = [
    "//autonomy/common:common",
    "//autonomy/commsgs:commsgs_cc",
]
