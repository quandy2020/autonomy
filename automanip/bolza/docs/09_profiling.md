# 9　性能剖析

- 顺序：库内计时 → `perf` / Valgrind（按需）
- 建议：Release

```bash
cmake -S automanip/bolza -B build/bolza -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-march=native -mtune=native"
cmake --build build/bolza -j"$(nproc)" --target bolza
# cartpole_demo 在 automanip/examples（上层工程）中构建
```

## 9.1　库内计时

```text
ddp { displayShortSummary   true }
```

- 或：`GetBenchmarkingInfo()`
- 排除首次 CppAD codegen
- 核对：`maxNumIterations` · `nThreads` · 时域 · 频率

## 9.2　系统工具

```bash
sudo perf record -g -p $(pgrep -n cartpole_demo)
sudo perf report -g 'graph,0.5,caller' -i perf.data

valgrind --tool=massif ./cartpole_demo 400
valgrind --tool=cachegrind ./cartpole_demo 400
```

- 可选编译：`-fno-omit-frame-pointer`
- 直接包装 `*_demo`，无需 ROS launch 前缀
