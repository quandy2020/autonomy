# C++ Get Started

## Overview

This tutorial shows how to use **automsgs** in a C++ project: include generated headers, create messages, and print them.

## Minimal example

Create a `main.cc`:

```cpp
#include <iostream>
#include <automsgs/msgs/geometry_msgs/Vector3.pb.h>
#include <automsgs/msgs/geometry_msgs/Pose.pb.h>

int main()
{
  automsgs::msgs::geometry_msgs::Vector3 point1;
  point1.set_x(1.0);
  point1.set_y(3.0);
  point1.set_z(5.0);

  automsgs::msgs::geometry_msgs::Vector3 point2;
  point2.set_x(2.0);
  point2.set_y(4.0);
  point2.set_z(6.0);

  std::cout << "Point1:\n" << point1.DebugString() << std::endl;
  std::cout << "Point2:\n" << point2.DebugString() << std::endl;

  automsgs::msgs::geometry_msgs::Pose pose;
  pose.mutable_position()->set_x(1.0);
  pose.mutable_position()->set_y(2.0);
  pose.mutable_position()->set_z(3.0);
  pose.mutable_orientation()->set_w(1.0);
  std::cout << "Pose:\n" << pose.DebugString() << std::endl;

  return 0;
}
```

## CMake

Create a `CMakeLists.txt` that finds and links `automsgs`:

```cmake
cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(my_automsgs_example)

find_package(automsgs QUIET REQUIRED)
# Or, if building inside the automsgs workspace, the target is just automsgs_proto:
# add_subdirectory(/path/to/automsgs)

add_executable(my_example main.cc)
target_link_libraries(my_example PRIVATE automsgs_proto)
```

If you install automsgs and use it as an external package, ensure your project has access to the installed `automsgs` CMake config and the `automsgs_proto` target (and its include directories).

## Build and run

```bash
mkdir build && cd build
cmake ..
make
./my_example
```

Example output:

```
Point1:
x: 1
y: 3
z: 5

Point2:
x: 2
y: 4
z: 6

Pose:
position {
  x: 1
  y: 2
  z: 3
}
orientation {
  w: 1
}
```

## Using the in-tree examples

The **automsgs** repository includes examples you can build from the same tree:

- **using_automsgs** – links to `automsgs_proto` and uses `Pose` and `String`.
- **generating_custom_msgs** – defines custom `.proto` messages that depend on automsgs and generates a library.

From the automsgs build directory:

```bash
cd /path/to/automsgs/build
cmake ..
make
./examples/using_automsgs/using_automsgs
./examples/generating_custom_msgs/generating_custom_msgs
```

See [Message Generation](message_generation.md) for custom message definitions and the generation pipeline.
