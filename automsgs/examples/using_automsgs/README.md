# Using automsgs in your project

This example shows how to use automsgs messages in your build.

1. Ensure the automsgs project is built (the `automsgs_proto` target and its include dir are set by the parent CMake).
2. Link the messages library to your executable:

```cmake
add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} PRIVATE automsgs_proto)
```

3. Include the generated headers by package path, e.g.:
   - `#include <automsgs/msgs/geometry_msgs/Pose.pb.h>`
   - `#include <automsgs/msgs/std_msgs/String.pb.h>`

For an example that generates custom message types that depend on automsgs, see the `generating_custom_msgs` example.
