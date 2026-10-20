<div align="center">

# AUTONOMY

**A modular intelligence framework for autonomous mobile robots**

Modern C++ · Autolink RT · Behavior trees · *No ROS runtime required*

[![Version](https://img.shields.io/badge/Version-0.2.0-2563EB?style=for-the-badge)](version.json)
[![C++](https://img.shields.io/badge/C%2B%2B-20-0F172A?style=for-the-badge&logo=cplusplus&logoColor=white)](CMakeLists.txt)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20ARM64-334155?style=for-the-badge&logo=linux&logoColor=white)](docs/source/02_Installation/00_guide.md)
[![License](https://img.shields.io/badge/License-Apache--2.0-0891B2?style=for-the-badge)](LICENSE)

<br/>

[Documentation](https://autonomy.readthedocs.io/en/latest/index.html)
&nbsp;·&nbsp;
[Install](docs/source/02_Installation/00_guide.md)
&nbsp;·&nbsp;
[Quick Start](docs/source/04_Running/02_quickstart.md)
&nbsp;·&nbsp;
[Architecture](docs/source/01_Instructions/03_system_architecture.md)

<br/>

<img src="images/autonomy_architecture.png" alt="Autonomy Architecture" width="920"/>

<br/>

`PERCEPTION`&nbsp;&nbsp;`LOCALIZATION`&nbsp;&nbsp;`MAPPING`&nbsp;&nbsp;`PLANNING`&nbsp;&nbsp;`CONTROL`&nbsp;&nbsp;`ORCHESTRATION`

</div>

<br/>

<table>
  <tr>
    <td align="center" width="33%">
      <h3>Runtime Independent</h3>
      <p>Node · Channel · Service · Action · Parameter<br/>without a ROS runtime dependency</p>
    </td>
    <td align="center" width="33%">
      <h3>Modular by Design</h3>
      <p>Stable plugins across algorithms,<br/>drivers, simulation, and visualization</p>
    </td>
    <td align="center" width="33%">
      <h3>Production Oriented</h3>
      <p>Docker · Ansible · observability<br/>x86_64 and ARM64 ready</p>
    </td>
  </tr>
</table>

---

## Gallery

<table>
  <tr>
    <td align="center" width="50%" valign="top">
      <h3>Localization · Argus LIVO</h3>
      <a href="docs/assets/localization/argus_mid25.mp4">
        <img src="docs/assets/localization/argus.png" alt="Argus LIVO" width="100%"/>
      </a>
      <p>
        <a href="docs/assets/localization/argus_mid25.mp4"><b>▶ Mid-360 demo</b></a>
        &nbsp;·&nbsp;
        <a href="autonomy/localization/README.md">Guide</a>
      </p>
      <sub>Cartographer · Atlas · Argus (ESIKF)</sub>
    </td>
    <td align="center" width="50%" valign="top">
      <h3>Simulation · Isaac + AutoViz</h3>
      <img src="autosim/images/isaac.gif" alt="Isaac Sim + AutoViz" width="100%"/>
      <p>
        <a href="autosim/README.md"><b>AutoSim</b></a>
        &nbsp;·&nbsp;
        <a href="autoviz/README.md">AutoViz</a>
      </p>
      <sub>Habitat · Fake · Isaac Sim + PhysX</sub>
    </td>
  </tr>
</table>

---

## Architecture

<div align="center">

<a href="docs/architecture/archify/autonomy-runtime.architecture.html">
  <img src="docs/architecture/archify/autonomy-runtime.architecture.gif" alt="Runtime architecture" width="920"/>
</a>

</div>

<br/>

<table>
  <tr>
    <td align="center" width="50%">
      <a href="docs/architecture/archify/navigation-task.workflow.html">
        <img src="docs/architecture/archify/navigation-task.workflow.gif" alt="Navigation workflow" width="100%"/>
      </a>
      <br/>
      <b>Navigation</b> — goal · plan · recover · cancel
    </td>
    <td align="center" width="50%">
      <a href="docs/architecture/archify/person-following.dataflow.html">
        <img src="docs/architecture/archify/person-following.dataflow.gif" alt="Person-following flow" width="100%"/>
      </a>
      <br/>
      <b>Person-following</b> — RGB · track · path · control
    </td>
  </tr>
</table>

<p align="center">
  <a href="docs/architecture/archify/autonomy-runtime.architecture.html">Runtime</a>
  &nbsp;·&nbsp;
  <a href="docs/architecture/archify/navigation-task.workflow.html">Navigation</a>
  &nbsp;·&nbsp;
  <a href="docs/architecture/archify/person-following.dataflow.html">Following</a>
  &nbsp;·&nbsp;
  <a href="docs/source/01_Instructions/03_system_architecture.md">System guide</a>
</p>

---

## Stack

<table>
  <tr>
    <td width="25%"><b>Runtime</b><br/><code>autolink/</code><br/><sub>SHM · RTPS · Action</sub></td>
    <td width="25%"><b>Messages</b><br/><code>automsgs/</code><br/><sub>Protobuf · codegen</sub></td>
    <td width="25%"><b>Localization</b><br/><code>autonomy/localization/</code><br/><sub>Cartographer · Atlas · Argus</sub></td>
    <td width="25%"><b>Perception</b><br/><code>autonomy/perception/</code><br/><sub>Detect · track · depth</sub></td>
  </tr>
  <tr>
    <td><b>Map / Plan / Ctrl</b><br/><code>map · planning · control</code><br/><sub>costmap · Theta* · MPPI</sub></td>
    <td><b>Tasks</b><br/><code>autonomy/task/</code><br/><sub>Behavior-tree nav</sub></td>
    <td><b>Hardware</b><br/><code>autodriver/</code><br/><sub>Camera · LiDAR · IMU</sub></td>
    <td><b>Sim / Viz</b><br/><code>autosim/ · autoviz/</code><br/><sub>Isaac · Qt OpenGL</sub></td>
  </tr>
</table>

---

## Quick Start

<table>
  <tr>
    <td width="50%" valign="top">

**① Clone & enter Docker**

```bash
git clone --recurse-submodules \
  https://github.com/quandy2020/autonomy.git
cd autonomy
export AUTONOMY_ENV="$PWD"
python3 docker/run_autonomy.py -p x86_64
```

</td>
    <td width="50%" valign="top">

**② Build & launch stack**

```bash
cd /workspace/autonomy
cmake -S . -B build -G Ninja
cmake --build build -j"$(nproc)"

export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

</td>
  </tr>
</table>

<details>
<summary><b>Argus localization</b> (autosim / Oxford Spires → <code>/imu</code> · <code>/points</code>)</summary>

```bash
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/localization/launch"
autolink launch start localization_argus.launch
# 另一终端：数据集 publish（见 autonomy/tools/datasets/README.md）
python -m autonomy.tools.datasets publish \
  --dataset spires --path /path/to/sequences/<name> --rate 1.0
```

</details>

<p align="center">
  <a href="docs/source/04_Running/02_quickstart.md">Quick Run</a>
  &nbsp;·&nbsp;
  <a href="docs/source/02_Installation/05_docker.md">Docker</a>
  &nbsp;·&nbsp;
  <a href="cmake/README.md">CMake</a>
  &nbsp;·&nbsp;
  <a href="autonomy/localization/README.md">Localization</a>
</p>

---

## Guides

<p align="center">
  <a href="autolink/README.md">Autolink</a>
  &nbsp;·&nbsp;
  <a href="automsgs/README.md">AutoMsgs</a>
  &nbsp;·&nbsp;
  <a href="autonomy/localization/README.md">Localization</a>
  &nbsp;·&nbsp;
  <a href="autodriver/README.md">AutoDriver</a>
  &nbsp;·&nbsp;
  <a href="autosim/README.md">AutoSim</a>
  &nbsp;·&nbsp;
  <a href="autoviz/README.md">AutoViz</a>
  &nbsp;·&nbsp;
  <a href="ansible/README.md">Ansible</a>
  &nbsp;·&nbsp;
  <a href="autocmake/README.md">autocmake</a>
</p>

<p align="center">
  <sub>
    <a href="docs/source/02_Installation/00_guide.md">Install</a> ·
    <a href="docs/source/04_Running/00_guide.md">Run</a> ·
    <a href="docs/source/06_Localization/index.rst">SLAM</a> ·
    <a href="docs/source/12_Simulation/index.rst">Sim</a> ·
    <a href="docs/source/17_Tasks/index.rst">Tasks</a> ·
    <a href="docs/source/19_FAQs/index.rst">FAQs</a> ·
    <a href="https://autonomy.readthedocs.io/en/latest/index.html">readthedocs</a>
  </sub>
</p>

---

<div align="center">

**Apache 2.0** · Inspired by Autoware · Cartographer · Navigation2 · ROS 2 · BehaviorTree.CPP

<br/>

*Autonomy — modular, observable, deployable robot intelligence*

</div>
