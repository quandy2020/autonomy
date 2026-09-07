# Autonomy Perception

> Open robots for everyone.

This directory contains the perception capabilities of the Autonomy project. Its subsystems are named after figures and concepts that reflect their roles in helping a robot perceive, understand, and navigate the world.

## Project Names

### Hestia — Home Scene Perception

**Hestia** is the Greek goddess of the hearth and home. The name represents a robot's ability to quietly and continuously understand the household environment it serves.

**HESTIA** stands for **Home Environment Semantic Tracking and Intelligent Awareness**.

Hestia is responsible for general home-scene perception, including:

- detecting people, pets, furniture, and everyday objects;
- semantic and instance segmentation;
- pose estimation and multi-object tracking;
- recognizing doors, stairs, floors, rooms, and traversable areas;
- providing a unified representation of the home for downstream robot tasks.

> **Hestia — Perceive the Home. Understand Every Scene.**
>
> 感知家园，理解万象。

Implementation lives under `autonomy/perception/hestia/` (see that directory's
`README.md` for topics, detector contract, and Orin/RK profiles).

### Ariadne — Autonomous Exploration and Mapping

**Ariadne** comes from Greek mythology. Her thread guided Theseus through the labyrinth and showed him the way back. The story closely reflects autonomous exploration and SLAM: entering an unknown environment, preserving a reliable path, constructing a map, and returning safely.

**ARIADNE** stands for **Autonomous Robotic Intelligence for Adaptive Discovery, Navigation, and Exploration**.

Ariadne is responsible for:

- autonomous exploration of unknown environments;
- SLAM and map construction;
- frontier discovery and exploration planning;
- coverage evaluation and unexplored-area selection;
- safe return and navigation using the generated map.

> **Ariadne — Explore the Unknown, Map the Way.**
>
> 探索未知，绘制归途。

### Shadow — Selected-Person Following

**Shadow** describes a robot that stays with a selected person as naturally and persistently as a shadow, even when the person changes direction, speed, or position.

**SHADOW** stands for **Selected Human Detection And Dynamic Observation Workflow**.

Shadow is responsible for:

- selecting and locking onto a target person;
- person detection, tracking, and re-identification;
- estimating the target's position and motion;
- generating safe following commands;
- recovering the target after temporary occlusion or loss.

> **Shadow — Choose Once. Follow Anywhere.**
>
> 选定一次，全程跟随。

## Naming Philosophy

The names are intentionally concise and meaningful:

- **Hestia** understands the home.
- **Ariadne** explores and maps the unknown.
- **Shadow** follows the chosen person.

Together, they form a perception foundation for robots that can understand their surroundings, explore independently, and interact naturally with people.
