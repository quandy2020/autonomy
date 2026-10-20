# 3. 编译问题

症状包括：CMake 报找不到 Protobuf/Ceres/OpenCV/Lua/gRPC、链接 autoviz 失败、编译被 OOM kill、或改 `BUILD_*` 后目标仍不出现。

先区分 **系统 apt 包**、**install_deps 第三方**、**BUILD_GRPC / BUILD_AUTOVIZ 开关** 三类根因；Bridge 与 autoviz 依赖不同，关其中一个不会自动关另一个。改 CMake 选项后宜清缓存重配。

## 问答

### Q: 找不到 Protobuf？

```bash
sudo apt install libprotobuf-dev protobuf-compiler
```

或走 `python3 -m install_deps --apt-only`。

### Q: 找不到 Ceres / OpenCV 等第三方？

```bash
python3 -m install_deps --thirdparty-only
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

中断可从 `--resume-from install_xxx.sh` 续装。

### Q: 找不到 Lua？

```bash
sudo apt install liblua5.3-dev
```

### Q: gRPC / Bridge 相关 CMake 报错？

需要 Bridge 时保持 **`-DBUILD_GRPC=ON`**（默认）。仅想跳过 Bridge 域：

```bash
cmake .. -DBUILD_GRPC=OFF
```

关后不会有 `autonomy.bridge`；FAQ 里「连不上 5005」先确认是否故意关的。

### Q: 找不到 autoviz / Ogre？

桌面可视化需 **`-DBUILD_AUTOVIZ=ON`**（默认常开）。板端省内存可 `-DBUILD_AUTOVIZ=OFF`，与 Bridge 无关。

### Q: 编译器过旧？

需要 C++17；Ubuntu 22.04 自带 GCC 11 一般够用。

### Q: OOM / killed？

```bash
ninja -j2
```

大模块并行度降到 2–4；build 目录放本地 SSD。

### Q: submodule 缺失？

```bash
git submodule update --init --recursive
```

### Q: 清理重配？

```bash
rm -rf build/* && cmake -G Ninja -B build && ninja -C build
```

改 `BUILD_*` 开关后建议清 CMake 缓存再配。
