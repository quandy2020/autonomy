# autoviz

**单一流程**：Autolink 上的 **automsgs（protobuf）** → **Foxglove Studio**（WebSocket）；可选 **MCAP** 落盘。

## 快速索引

| 项 | 说明 |
|----|------|
| **构建** | 本目录 `CMakeLists.txt`，目标 `autoviz`、`autoviz_core` |
| **配置** | `config/autoviz.pb.conf`（文本 proto，定义见 `proto/autoviz_conf.proto`）；路径解析与 autolink 一致，可设 **`AUTOLINK_CONF_PATH`**（含配置所在目录） |
| **运行时结构** | `core/README.md` |
| **automsgs ↔ Foxglove** | `core/convert/README.md` |

## 维护与常见调整

1. **新增 / 变更 `.proto` 后**  
   重新生成注册表头文件（字符串池 + 有序 `Row` 表）：
   ```bash
   python3 src/autonomy/autoviz/core/convert/tools/generate_automsgs_foxglove_registry.py
   ```
   或在已配置 CMake 的 build 目录：`cmake --build . --target autoviz_regenerate_foxglove_registry`

2. **Foxglove 未链接成功**  
   需先有 vendored 的 `foxglove-sdk`（通常随主工程 `_deps/foxglove-src`）。否则 WebSocket 服务可能无法编译或运行。

3. **MCAP**  
   未找到 Mcap 包时仍可通过编译，录制功能由 `AUTOVIZ_HAS_MCAP` 控制；可在 `autoviz.pb.conf` 里关闭 `mcap { enabled: false }`。

4. **可选后续优化（未改代码，按需做）**  
   - `convert_automsgs_body_3d.cpp` 体量较大，可按消息包拆文件。  
   - `SchemaConverter` 与 Foxglove 通道 schema 可随 SDK 升级再对齐。  
   - 需要时为 `FindFoxgloveRule` / `ConvertMessageToFoxglove` 增加单测。

## 示例

`example/datas_publisher.cpp`（`AUTOVIZ_BUILD_EXAMPLE=ON` 时构建）。

## 单元测试

```bash
# 在已包含 autoviz 子目录的 CMake 工程里打开测试并编译：
cmake <your-build-dir> -DAUTOVIZ_BUILD_TEST=ON
cmake --build <your-build-dir> --target autoviz_registry_test
cd <your-build-dir> && ctest -R autoviz_registry_test
```

依赖 **GTest**；`autoviz_registry_test` 校验 `FindFoxgloveRule` 与生成表一致（不拉 Foxglove WebSocket）。
