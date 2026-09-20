# ZSL-1W SDK

## Description

SDK for ZSongLing (中山松灵) ZSL-1W quadrupedal robot.

## Source

Original SDK location: `/home/lichengmin/Project/zsibot_sdk`

Copied on: 2025-12-12

## Contents

- **include/zsl-1w/**: C++ header files
  - `highlevel.h`: High-level control API

- **lib/x86_64/**: x86_64 architecture libraries
  - `libmc_sdk_zsl_1w_x86_64.so`: Main SDK library

- **lib/aarch64/**: ARM64 architecture libraries  
  - `libmc_sdk_zsl_1w_aarch64.so`: Main SDK library

## API Overview

The `HighLevel` class provides:
- Robot connection and initialization
- Movement control (forward, backward, lateral, rotation)
- State control (stand up, lie down, passive mode)
- Special actions (crawl, climb, shake hand, attitude control)
- Sensor data acquisition:
  - Position, velocity (world frame and body frame)
  - Quaternion, RPY (Roll-Pitch-Yaw)
  - Body acceleration and gyroscope
  - Battery power and control mode
  - Joint positions, velocities, and torques

## Usage in CMakeLists.txt

```cmake
# Set SDK path
set(ZSL1W_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third-party/zsl1w")

# Detect architecture
set(ARCH "x86_64")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH "aarch64")
endif()

# Find library
find_library(ZSL1W_LIB 
    NAMES "mc_sdk_zsl_1w_${ARCH}"
    PATHS "${ZSL1W_SDK_PATH}/lib/${ARCH}"
    NO_DEFAULT_PATH
)

# Include directories
include_directories(${ZSL1W_SDK_PATH}/include)

# Link library
target_link_libraries(your_target ${ZSL1W_LIB})
```

## Network Configuration

Default robot IP: `192.168.234.1`
Default local port: `43988`

## License

Refer to the original SDK license from the vendor.

## Contact

For SDK issues, contact the vendor: ZSongLing Robotics
