# Creating custom automsgs message types

This example shows how to define your own message types that depend on automsgs.

## Layout

Create a `proto/` tree that matches your package name. For package `automsgs.custom_msgs`:

- `proto/automsgs/custom_msgs/foo.proto`
- `proto/automsgs/custom_msgs/bar.proto`
- `proto/automsgs/custom_msgs/baz.proto`

In each `.proto` file, set the package and import automsgs types as needed:

```protobuf
syntax = "proto3";
package automsgs.custom_msgs;

import "automsgs/msgs/std_msgs/header.proto";

message Foo {
  double value = 1;
}
```

## Building from automsgs root

This example must be built as part of the automsgs project so that `AUTOMSGS_PROTO_INCLUDE_DIR` and `AUTOMSGS_PROTO_GEN_DIR` are set by the main `proto/` build. From the automsgs build directory:

```bash
cd build && cmake .. && make
```

The example will:

1. Copy its `proto/` tree into the build directory.
2. Run `protoc` with `-I` pointing to the main automsgs proto include and the example proto copy.
3. Build a library `generating_custom_msgs-msgs` and the executable `generating_custom_msgs`.

## Running

```bash
./generating_custom_msgs
```

This prints the descriptor and a populated `BazStamped` message.
