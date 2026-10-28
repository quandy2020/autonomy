# 03 构建与 CMake

## autonomy 工程构建

在 `SpaceHero` 容器中执行：

```bash
docker exec -w /workspace/autonomy SpaceHero \
  colcon build --packages-up-to autonomy
```

Atlas 生成以下主要 target：

- `atlas_core`：SLAM 核心静态库。
- `atlas_automsgs_bridge`：AutoMsgs 数据适配。
- `autonomy_localization_atlas`：Autolink 节点库。
- `autonomy::localization_atlas`：供其他模块链接的别名。

## 构建选项

- `ATLAS_BUILD_EXAMPLES`：构建数据集示例，默认 `OFF`。
- `ATLAS_BUILD_TESTS`：构建并注册单元测试，默认 `OFF`。
- `ATLAS_ENABLE_OPENMP`：启用双目 ORB 的有界并行，默认 `ON`。

OpenMP 不可用时会打印警告并自动使用双线程 ThreadPool 回退。Atlas 不在
Matcher、ORB 金字塔或 g2o 内嵌额外 OpenMP 并行。

## Cartographer 风格的 CMake 组织

Atlas 参考 Cartographer 的构建布局：

- 根 `CMakeLists.txt` 负责 options、依赖查找、源码分类、targets、链接和
  安装规则。
- `cmake/functions.cmake` 只保存公共辅助函数，包括项目初始化、target
  公共属性、示例和测试创建。
- `cmake/modules/` 只保存 `FindAutoMsgs.cmake` 和 `FindAutolink.cmake`，
  为安装包未导出 imported target 时提供兼容回退。g2o 与 FBow 直接使用
  SpaceHero 中 `/usr/local` 安装的 CMake CONFIG 包。

根 CMake 使用 `GLOB_RECURSE CONFIGURE_DEPENDS` 自动发现 Atlas 自有
`.hpp/.cpp`，再明确剔除测试、AutoMsgs bridge 和 node 源文件。该方式是
Cartographer 文件自动发现机制在现代 CMake 中的实现，不需要旧版的 shell
`find/diff` 变更检测 target。

`atlas_initialize_project()` 会把 `cmake/modules` 加入
`CMAKE_MODULE_PATH`，因此 Atlas 在 autonomy 子工程和独立配置时使用
相同的依赖查找逻辑。为避免污染 autonomy 父工程，编译选项只设置在 Atlas
targets 上，不全局覆盖 `CMAKE_BUILD_TYPE` 或 C++ flags。

## 独立配置

独立配置仍需要 AutoMsgs、Autolink、OpenCV、Eigen、OpenSSL、FBow、g2o
和 Threads 的开发包。SpaceHero 将 g2o 与 FBow 的 CONFIG 包安装在
`/usr/local`；独立配置时建议显式加入该前缀：

```bash
cmake -S autonomy/localization/atlas \
  -B build/atlas \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DATLAS_BUILD_EXAMPLES=ON
cmake --build build/atlas -j
```

若机器同时安装了多个 OpenCV 版本，应显式设置包含 `features2d` 和
`calib3d` 的配置目录，例如：

```bash
-DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

如需验证无 OpenMP 回退，可在独立 build 目录增加
`-DATLAS_ENABLE_OPENMP=OFF`，不要覆盖用于性能基准的 build cache。
