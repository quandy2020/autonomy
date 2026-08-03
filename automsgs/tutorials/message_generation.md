# Message Generation

## Overview

**automsgs** uses [Protocol Buffers](https://protobuf.dev) (protobuf) to define message structures. The build system compiles `.proto` files into C++ and Python so you can use them in your application or create custom messages that depend on automsgs.

## Message definitions

### File layout

Message definitions live under the [proto](https://github.com/your-org/automsgs/tree/main/proto) directory:

- `proto/msgs/` – message definitions (e.g. `geometry_msgs`, `std_msgs`, `sensor_msgs`)
- `proto/srvs/` – service definitions
- `proto/actions/` – action definitions (e.g. `nav_actions.proto`)
- `proto/rpcs/` – RPC service definitions

Package names follow the path: e.g. `automsgs.msgs.geometry_msgs` maps to the C++ namespace `automsgs::msgs::geometry_msgs` and to Python `automsgs.msgs.geometry_msgs`.

### Proto files

Messages are defined in `.proto` files using **proto3** syntax. A minimal example:

```proto
syntax = "proto3";

package automsgs.msgs.geometry_msgs;

message Vector3 {
  double x = 1;
  double y = 2;
  double z = 3;
}
```

To use another message type, import its `.proto` and reference it by package and type:

```proto
syntax = "proto3";

package automsgs.msgs.geometry_msgs;

import "automsgs/msgs/geometry_msgs/Point.proto";
import "automsgs/msgs/geometry_msgs/Quaternion.proto";

message Pose {
  Point position = 1;
  Quaternion orientation = 2;
}
```

Field numbers must be unique within the message and follow [protobuf rules](https://protobuf.dev/programming-guides/proto3/#assigning).

## Generation pipeline

1. **Proto include layout**  
   The build copies `proto/msgs` and `proto/srvs` into a build-time directory so that `import "automsgs/msgs/..."` is resolved.

2. **Per-file generation**  
   For each `.proto` file, the build runs:
   - **protoc** – generates `.pb.h`, `.pb.cc`, and Python `_pb2.py`.
   - **automsgs_msgs_generate.py** – runs protoc and then:
     - Copies the generated `.pb.h` into a `details/` subdirectory (for a gz-msgs–style layout).
     - Writes a `.pb_index` file (message type name) for tooling.

3. **Library**  
   All generated `.pb.cc` files are compiled into the `automsgs_proto` library. Headers are exposed under the same path as the package (e.g. `automsgs/msgs/geometry_msgs/Vector3.pb.h`).

4. **Optional: MessageTypes.hh**  
   The script `automsgs_generate_factory.py` can generate a single header that includes all generated message headers (no Factory/registration in automsgs).

## Custom message generation

You can define your own messages that depend on automsgs. The repository includes a full example in **examples/generating_custom_msgs**.

### Directory structure

Match your package name with the directory structure. For package `automsgs.custom_msgs`:

```text
your_project/
  proto/
    automsgs/
      custom_msgs/
        foo.proto
        bar.proto
        baz.proto
```

### Example proto files

**foo.proto**

```proto
syntax = "proto3";

package automsgs.custom_msgs;

import "automsgs/msgs/std_msgs/Header.proto";

message Foo {
  double value = 1;
}

message FooStamped {
  automsgs.msgs.std_msgs.Header header = 1;
  Foo foo = 2;
}
```

**bar.proto**

```proto
syntax = "proto3";

package automsgs.custom_msgs;

import "automsgs/msgs/std_msgs/Header.proto";

message Bar {
  double value = 1;
}
```

**baz.proto**

```proto
syntax = "proto3";

package automsgs.custom_msgs;

import "automsgs/msgs/std_msgs/Header.proto";
import "automsgs/custom_msgs/foo.proto";
import "automsgs/custom_msgs/bar.proto";

message Baz {
  Foo foo = 1;
  Bar bar = 2;
}

message BazStamped {
  automsgs.msgs.std_msgs.Header header = 1;
  Baz baz = 2;
}
```

### Building custom messages

When building **inside** the automsgs tree (e.g. as in `examples/generating_custom_msgs`):

1. Copy your `proto/` tree into the build directory.
2. Run **protoc** with `-I` pointing to both the automsgs proto include directory and your copied proto root.
3. Compile the generated `.pb.cc` into a library and link your executable to it and to `automsgs_proto`.

See **examples/generating_custom_msgs/CMakeLists.txt** for a full CMake example that uses `AUTOMSGS_PROTO_INCLUDE_DIR` and `AUTOMSGS_PROTO_GEN_DIR` set by the main automsgs `proto/` build.

### Using custom messages in C++

Include the generated headers and link the library that contains them:

```cpp
#include <automsgs/custom_msgs/foo.pb.h>
#include <automsgs/custom_msgs/baz.pb.h>

int main() {
  automsgs::custom_msgs::BazStamped msg;
  msg.mutable_header()->set_frame_id("world");
  msg.mutable_baz()->mutable_foo()->set_value(1.0);
  msg.mutable_baz()->mutable_bar()->set_value(2.0);
  return 0;
}
```

```cmake
add_executable(my_app main.cc)
target_link_libraries(my_app PRIVATE my_custom_msgs_lib automsgs_proto)
```

## Python

Generated Python modules are installed under the configured Python path (e.g. `lib/python`). Use them as:

```python
from automsgs.msgs.geometry_msgs.Vector3_pb2 import Vector3
msg = Vector3()
msg.x = 1.0
msg.y = 2.0
msg.z = 3.0
print(msg.SerializeToString())
```

See the **python/** directory and the **basic_TEST.py** test for more.
