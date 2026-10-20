# 9. Autoviz

顶层包 `autoviz/`，CMake 开关 `BUILD_AUTOVIZ`。不依赖 ROS，直接走 Autolink；界面是 Qt + Ogre，Linux / macOS / Windows 都能编。长文在包内 `autoviz/docs/`。

和 Foxglove、RViz2 的差别：Autoviz 是桌面进程吃 Autolink；Foxglove 偏浏览器 / WS；RViz2 绑 ROS 2。

```bash
colcon build --packages-select autoviz --cmake-args -DAUTOLINK_BUILD_PYTHON=ON
source install/setup.bash && autoviz
```

也可以在编 autonomy 时加 `-DBUILD_AUTOVIZ=ON`。Linux 装 Qt6 开发包；macOS 看 `autoviz/deploy/macos/README.md`。
