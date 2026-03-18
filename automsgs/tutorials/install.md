# Installation

Next: [C++ Get Started](cppgetstarted.md)

This document describes how to install and build **automsgs** from source.

## Prerequisites

- CMake >= 3.22.1
- C++17 compiler
- [Protocol Buffers](https://protobuf.dev/) (libprotobuf-dev, protobuf-compiler)
- Python 3 (for code generation scripts)

### Ubuntu / Debian

```bash
sudo apt-get install -y build-essential cmake libprotobuf-dev protobuf-compiler libprotoc-dev python3
```

### macOS

```bash
brew install cmake protobuf python3
```

## Building from source

From the repository root (or the `automsgs` package root):

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install   # optional
```

To install to a custom prefix:

```bash
cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install
make -j$(nproc)
make install
```

## Uninstall

If you used the default install and have the build directory:

```bash
cd build
sudo make uninstall
```

## Testing

Build and run tests:

```bash
cd build
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)
ctest
```

To disable tests during configuration:

```bash
cmake .. -DBUILD_TESTING=OFF
```

## Documentation

- [C++ Get Started](cppgetstarted.md) – use messages in a C++ project
- [Message Generation](message_generation.md) – proto layout and custom messages
