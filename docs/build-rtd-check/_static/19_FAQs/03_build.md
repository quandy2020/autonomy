# 3. 编译问题

### Q: `Could NOT find Protobuf`？

```bash
sudo apt install libprotobuf-dev protobuf-compiler
# 或运行 install_protobuf.sh
```

### Q: `Could NOT find Ceres`？

```bash
# 运行第三方安装
python3 scripts/install_dependency.py --thirdparty-only
# 确认 /usr/local/lib 在链接路径
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Q: `Could NOT find Lua`？

```bash
sudo apt install liblua5.3-dev
```

### Q: gRPC 相关 CMake 错误？

安装 gRPC 或关闭选项：

```bash
cmake .. -DBUILD_GRPC=OFF
```

### Q: C++17 编译错误 / 编译器过旧？

升级 GCC 至 11+（Ubuntu 22.04 默认满足）。

### Q: 编译时 OOM / 编译器被 kill？

限制并行度：

```bash
ninja -j2
```

### Q: submodule 缺失？

```bash
git submodule update --init --recursive
```

### Q: 清理后重配？

```bash
cd build && rm -rf * && cmake -G Ninja .. && ninja
```
