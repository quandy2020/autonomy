# 3. 编译问题

### Q: 找不到 Protobuf？

```bash
sudo apt install libprotobuf-dev protobuf-compiler
```

### Q: 找不到 Ceres？

```bash
python3 -m install_deps --thirdparty-only
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Q: 找不到 Lua？

```bash
sudo apt install liblua5.3-dev
```

### Q: gRPC CMake 错？

```bash
cmake .. -DBUILD_GRPC=OFF
```

### Q: 编译器过旧？

GCC 11+（Ubuntu 22.04 默认即可）。

### Q: OOM？

```bash
ninja -j2
```

### Q: submodule 缺失？

```bash
git submodule update --init --recursive
```

### Q: 清理重配？

```bash
rm -rf build/* && cmake -G Ninja -B build && ninja -C build
```
