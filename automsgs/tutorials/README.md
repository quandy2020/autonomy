# automsgs Tutorials

Tutorials for building, using, and extending **automsgs**.

| Tutorial | Description |
|----------|-------------|
| [Installation](install.md) | Build and install automsgs from source; run tests. |
| [C++ Get Started](cppgetstarted.md) | Use automsgs messages in a C++ project (include headers, link library, minimal example). |
| [Message Generation](message_generation.md) | Proto layout, generation pipeline, and custom message definitions. |

The in-tree **examples** are also useful:

- `examples/using_automsgs` – link to `automsgs_proto` and use `Pose` / `String`.
- `examples/generating_custom_msgs` – define custom `.proto` messages that depend on automsgs and build a library and executable.

Runtime message registration and descriptor paths are covered by `DynamicFactory`.
