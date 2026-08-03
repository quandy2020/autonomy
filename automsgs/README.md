# automsgs

Protocol Buffer message and service definitions for the autonomy stack, with C++/Python code generation and tooling. The layout and build pipeline are inspired by [gz-msgs](https://github.com/gazebosim/gz-msgs).

## Features

- **Proto definitions**: ROS-style messages, services, and actions under `proto/msgs/`, `proto/srvs/`, and `proto/actions/` (e.g. `geometry_msgs`, `sensor_msgs`, `std_msgs`, `nav_msgs`). All `.proto` filenames use **lowercase and underscores** (snake_case).
- **Code generation**: C++ (`.pb.h` / `.pb.cc`) and Python (`_pb2.py`) generated at build time via `protoc` and the `automsgs_msgs_generate.py` script; optional `details/` layout and `MessageTypes.hh` factory header.
- **Core library**: `automsgs_core` provides version config, install prefix, and **Factory** / **MessageFactory** / **DynamicFactory** for creating messages by type name and loading descriptors.
- **CLI**: `automsgs-msgs` lists installed message types and prints proto file contents (`--list`, `--info TYPE`).
- **Examples**: `using_automsgs` (link and use messages) and `generating_custom_msgs` (custom protos depending on automsgs).
- **Tests**: C++ integration tests and Python tests (pytest); optional performance/regression placeholders.

## Prerequisites

- CMake ≥ 3.22.1, C++17 compiler
- [Protocol Buffers](https://protobuf.dev/) (libprotobuf, protoc, libprotoc)
- Python 3 (for generation scripts; for Python tests: `pip install protobuf`)

## Build and install

```bash
cd /path/to/automsgs
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install   # optional
```

Custom install prefix:

```bash
cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install
make -j$(nproc)
make install
```

## Directory layout

| Directory    | Description |
|-------------|-------------|
| `proto/`    | `.proto` sources (`msgs/`, `srvs/`, `rpcs/`, `actions/`); build generates C++/Python under `build/proto/gen/`. |
| `core/`     | `automsgs_core` library (config, Factory, MessageFactory, DynamicFactory); `cmd/` for `automsgs-msgs` CLI; `generator/` and `generator_lite/` optional protoc plugins. |
| `tools/`    | `automsgs_msgs_generate.py`, `automsgs_generate_factory.py` (installed to `bin`). |
| `conf/`     | YAML config for CLI (e.g. `msgs1.yaml` installed to `share/automsgs/`). |
| `python/`   | Python package and tests. |
| `examples/`| `using_automsgs`, `generating_custom_msgs`. |
| `test/`     | C++ integration (and optional performance/regression) tests. |
| `tutorials/`| Install, C++ get started, message generation. |

## Using in C++

Link against `automsgs_proto` and include generated headers (snake_case filenames):

```cpp
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>

automsgs::msgs::geometry_msgs::Pose pose;
pose.mutable_position()->set_x(1.0);
```

See `examples/using_automsgs` and [tutorials/cppgetstarted.md](tutorials/cppgetstarted.md).

## Using in Python

Ensure generated Python is on `PYTHONPATH` (e.g. install path or `build/proto/gen/python`). Message types use snake_case module names:

```python
from automsgs.msgs.geometry_msgs.vector3_pb2 import Vector3
msg = Vector3()
msg.x, msg.y, msg.z = 1.0, 2.0, 3.0
```

See [python/README.md](python/README.md).

## CLI

After install, run:

```bash
automsgs-msgs --help
automsgs-msgs --list
automsgs-msgs --info automsgs.msgs.geometry_msgs.Pose
```

## Examples and tutorials

- **Examples**: [examples/using_automsgs](examples/using_automsgs), [examples/generating_custom_msgs](examples/generating_custom_msgs)
- **Tutorials**: [tutorials/README.md](tutorials/README.md) (install, C++ get started, message generation)

## Running tests

```bash
cd build
ctest
```

Python tests require `pip install protobuf` if the environment does not provide it.

## License

Copyright 2025 The Openbot Authors. Licensed under the Apache License, Version 2.0.
