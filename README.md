# Autonomy

![Autonomy system architecture](./images/autonomy_architecture.png)

**Autonomy** is an autonomy software framework for ground mobile robots: modular C++17 libraries, centralized Lua/Protobuf configuration, behavior-tree task orchestration, and a full 2D navigation stack. The communication runtime embeds [autolink](https://github.com/quandy2020/autolink) and **does not depend on ROS/ROS 2**; optional gRPC Bridge and message conventions enable interoperability with external ecosystems.

Online docs: [Read the Docs](https://autonomy.readthedocs.io/en/latest/index.html)

## Features

| Area | Capabilities |
|:-----|:---------------|
| **Communication** | Autolink (Node / Channel / Service / Action / Component) |
| **Messages** | AutoMsgs (Protobuf messages and tooling) |
| **Drivers** | AutoDriver (camera / lidar / CAN / chassis, and more) |
| **Perception** | Vision backbone, depth, person following, open-vocabulary scene perception, and related components |
| **Localization** | Atlas visual SLAM; Cartographer lidar SLAM; |
| **Mapping** | `costmap_2d` / grid_map |
| **Planning** | Global planners such as NavFn / Dijkstra / Theta\* |
| **Control** | Controllers such as MPPI / NMPC / Graceful / Pure Pursuit |
| **Prediction** | Trajectory / behavior prediction modules |
| **Tasks** | Behavior-tree apps: navigation / following / exploration / teleop / docking |
| **Simulation** | AutoSim |
| **Visualization** | AutoViz; Foxglove Bridge |
| **Deployment** | Docker (x86_64 / aarch64); Ansible bare-metal distribution |

For a fuller layered view, see [System architecture](docs/source/01_Instructions/03_system_architecture.md).

## Requirements

- **Recommended**: Docker (dependencies bundled; Ubuntu 22.04)
- **From source**: Ubuntu 22.04; GCC 11+ / Clang (C++17)

## Quick start

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy
```

### 2. Build (pick one)

#### Option A: Docker (recommended)

```bash
export AUTONOMY_ENV=/path/to/autonomy
python3 docker/run_autonomy.py -p x86_64
```

Inside the container:

```bash
cd /workspace/autonomy
mkdir -p build && cd build
cmake -G Ninja ..
ninja
```

More options: [Docker docs](docs/source/02_Installation/05_docker.md).

#### Option B: Host build

```bash
cd scripts && python3 -m install_deps
cd ..
mkdir -p build && cd build
cmake -G Ninja ..
ninja
```

Build details: [Build guide](docs/source/02_Installation/06_build.md).

### 3. Ansible deploy (optional)

Use Ansible when you do not want to compile by hand on every robot. Two common flows:

1. **Install on this machine**: build and install locally
2. **Fleet install**: package once, push to multiple robots; later config-only updates can skip a full reinstall

See [`ansible/README.md`](ansible/README.md) for details.

```bash
cd ansible
pip install "ansible>=8,<10"   # first time only

# Build and install on this host
./deploy.sh build

# Package, push to robots; use push later for config-only updates
../scripts/package_autonomy_artifact.sh --output ../dist/autonomy.tar.gz
./deploy.sh deploy robots -e autonomy_artifact_path=$PWD/../dist/autonomy.tar.gz
./deploy.sh push robots
```

| Command | Description |
|---------|-------------|
| `./deploy.sh build` | Build and install on this host |
| `./deploy.sh deploy robots -e …` | Push the install package to robots |
| `./deploy.sh push robots` | Update config and restart services only |
| `./deploy.sh check robots` | Pre-deploy readiness check |

Before multi-robot push: set robot IPs in `inventory/robots/hosts.yml` and configure passwordless SSH.

## CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_GRPC` | ON | gRPC Bridge |
| `BUILD_TEST` | ON | Unit tests |
| `BUILD_DOCS` | ON | Sphinx documentation |

## Layout

```text
autonomy/
├── autonomy/     # Algorithms system (perception/localization/map/planning/control/tasks …)
├── autolink/     # Communication runtime (submodule)
├── autodriver/   # Sensor and chassis drivers
├── automsgs/     # Protobuf message definitions and tools
├── autosim/      # Simulation
├── autoviz/      # Visualization
├── config/       # Runtime config (Lua / YAML / behavior trees, …)
├── cmake/        # Build modules
├── docker/       # Images and dependency scripts
├── ansible/      # Bare-metal deployment
├── docs/         # Sphinx docs and architecture diagrams
├── images/       # Repo-level diagrams
├── scripts/      # install_deps / packaging helpers
└── CMakeLists.txt
```

## Links

- Docs: [Online documentation](https://autonomy.readthedocs.io/en/latest/index.html)
- Getting started: [docs/source/01_Instructions/00_guide.md](docs/source/01_Instructions/00_guide.md)
- License: [Apache 2.0](LICENSE)

## Acknowledgments

- [Autoware Universe](https://github.com/autowarefoundation/autoware.universe)
- [Cartographer](https://cartographer-project.org/)
- [Navigation2](https://github.com/ros-navigation/navigation2)
- [ROS 2](https://github.com/ros2)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
