^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autonomy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-11-13)
------------------
* **Docs**

  * Refresh top-level README to a Concat-style layout: architecture hero,
    demos, merged Build path, Architecture workflows, and Guides.
  * Use lightweight AutoSim / cover assets for GitHub README load time.
* **Build / CMake**

  * Bump ``version.json`` / package version to ``0.4.0``.
  * Ignore control policy model artifacts under ``autonomy/control/conf/models/``.

0.3.0 (2026-11-04)
------------------
* **Architecture**

  * Split the tree into sibling packages under one Autocmake umbrella: ``automsgs``, ``autolink``, ``autodriver``, optional ``autoviz``, plus ``autosim`` and ``automanip``.
  * Move Atlas into ``autonomy/localization/atlas``; keep Cartographer and Argus LIVO as first-class localization backends.
  * Fold manipulation out of ``autonomy/manipulation`` into ``automanip`` (MoveIt-shaped stack + Bolza OCP: DDP / SQP / IPM / SLP / MPC).
* **Localization / mapping**

  * Atlas RGB-D inertial tracking with AutoSim: dense point cloud, occupancy grid, rolling navigation map, and ``map`` → ``odom`` TF.
  * Rebuild fused voxels after BA / IMU init / scale refinement (``InformNewBigChange``) so dense clouds do not leave ghost walls.
  * Hold the last IMU-RGBD pose when parked instead of resetting the atlas map after TrackLocalMap failure.
* **Visualization / simulation**

  * Native AutoViz (Qt + Ogre, Autolink, no ROS): occupancy, dense cloud, GPS overlay, robot model.
  * AutoSim sensor–actuator bridge (Habitat / Fake / Isaac + PhysX) with Atlas RGB-D+IMU validation path.
* **Build / CMake**

  * ``version.json`` / package version ``0.3.0``; ``BUILD_AUTOVIZ``; C++20; Autocmake domain options ``AUTONOMY_BUILD_<MOD>``.
* **Docs**

  * Refresh architecture, localization Atlas+AutoSim quickstart, AutoViz / AutoSim / Automanip guides.

0.2.0 (2026-07-17)
------------------
* **Architecture**

  * Embed ``autolink`` as a git submodule and link it into ``libautonomy`` (process entrypoints: ``localization`` / ``planning`` / ``control`` / ``task``).
  * Merge BehaviorTree plugins into ``libautonomy`` (static ``BT_REGISTER_NODES`` registry); remove separate ``libautonomy_task_*`` / ``cmake/plugin/`` shared plugins.
  * Replace Taskflow in ``TaskScheduler`` with ``std::thread``-based pipelines; drop ``cmake/taskflow.cmake``.
* **Control**

  * Restore controller protos and implementations: MPPI, Graceful, Pure Pursuit (wired via ``controller_options.proto``).
* **Build / CMake**

  * Split helpers: ``autonomy_sources`` / ``protobuf`` / ``autonomy_deps`` / ``autonomy_tests`` / ``autonomy_install`` / ``autolink_embed``.
  * Add ``BUILD_TOOLS``; link ``Threads`` and ``nlohmann_json``; drop unused ADOL-C ``find_package``.
  * Optional foxglove-sdk for visualization sources; always build ``autonomy_foxglove_bridge`` binary when configured.
* **Docs**

  * Refresh root ``README.md`` for current modules, binaries, and submodule clone flow.

0.1.1 (2026-05-19)
------------------
* Replace Boost with C++17 standard library in ``autonomy/`` (``std::optional``, ``std::mutex``, ``std::unordered_map``, ``std::function``, custom ``VoidSignal`` for tf2).
* Remove unused gzip helpers and ``find_package(Boost)`` from CMake.
* Remove Autolink middleware dependency from ``libautonomy`` (CMake-only build path).
* **Tasks / behavior tree**

  * Replace ROS/Autolink ``SimpleActionServer`` with in-process ``BtActionServer`` (``Run()``, in-memory goal/preempt/cancel).
  * Inline minimal action/service client types in ``behavior_tree_action_node.hpp`` and ``behavior_tree_service_node.hpp``; remove ``client_wrapper.hpp`` / ``server_wrapper.hpp``.
  * Strip Autolink ``Node`` / ``Reader`` / ``Client`` / ``Service`` from BT plugins; use ``GetInputOrBlackboard`` for ``global_frame`` / ``robot_base_frame``.
  * Simplify selector nodes and ``pause_resume_controller`` (no topic/service subscribers).
  * ``BtNavigator`` takes ``TaskOptions`` only; ``BehaviorTreeNavigator`` includes ``task_options.pb.h``; TF buffer uses singleton via non-owning ``shared_ptr``.
  * Fix ``PopulateInternalError`` proto enum conversion; ``NavigateToPoseNavigator`` blackboard read uses ``nodiscard``-safe ``get()``.
  * Add missing ``autonomy/common/log.hpp`` includes in BT plugin sources.
* **Control / planning**

  * ``ControllerServer`` / ``PlannerServer`` run without Autolink action servers (stub or direct planner wiring).
  * ``GracefulController`` no longer publishes via Autolink topics.
* **Map / transform / common**

  * Costmap layers and filters compile without Autolink publishers.
  * Remove Bazel ``BUILD`` / ``BUILD.bazel`` files under autonomy modules; drop ``FindOMPL.cmake`` and Docker Bazel/FastDDS install scripts.
* **Build**

  * ``tasks`` sources are always compiled into ``libautonomy`` (no Autolink-based source exclusion).
  * Require ``behaviortree_cpp`` via CMake ``find_package``.

0.0.3 (2025-10-16)
------------------
* Fix cyclonedds client and service test
* Fix cyclonedds clock test
* Fix cyclonedds service stats test
* Fix cyclonedds client stats test
* Fix cyclonedds client stats test
