# Autonomy Atlas visual SLAM

Atlas is the autonomy-owned ORB-SLAM3 integration. It is self-contained and
does not depend on the repository's Voxel-SLAM or Argus implementations.

The public C++ entry point is `atlas/map_builder.hpp`. The runtime supports
monocular, stereo, RGB-D, monocular-inertial, stereo-inertial and
RGB-D-inertial tracking. AutoMsgs conversion and Autolink integration live in
`atlas/message_adapter.*` and `atlas/atlas_node.*`.

## Documentation

Read the documents in numeric order:

1. [Overview](docs/01-overview.md)
2. [Architecture](docs/02-architecture.md)
3. [Build and CMake](docs/03-build-and-cmake.md)
4. [Configuration](docs/04-configuration.md)
5. [AutoMsgs integration](docs/05-automsgs-integration.md)
6. [Testing and benchmarking](docs/06-testing-and-benchmarking.md)
7. [Development guide](docs/07-development-guide.md)
8. [Third-party software](docs/08-third-party.md)

## Quick build

Atlas is built as part of the `autonomy` package:

```bash
docker exec -w /workspace/autonomy SpaceHero \
  colcon build --packages-up-to autonomy
```

The default build enables bounded OpenMP parallelism for stereo ORB
extraction. See [Build and CMake](docs/03-build-and-cmake.md) for all options
and standalone configuration.

ORB-SLAM3-derived code is GPLv3. See [COPYING](COPYING) and
[THIRD_PARTY.md](THIRD_PARTY.md).
