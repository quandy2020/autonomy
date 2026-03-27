# `core/convert/` — automsgs ↔ Foxglove

目录与文件名为 **`snake_case`**；公共 API 在 **`autoviz::converter`**。函数/方法名为 **PascalCase**（Google C++ 常见写法）。

## 目录结构

```
convert/
├── registry/          # proto 全名 → 策略与 canonical 名
│   ├── automsgs_foxglove_registry.hpp / .cpp   # 查找：vector + 二分
│   └── automsgs_foxglove_registry_data.hpp     # 脚本生成：字符串池 + constexpr Row[]
├── encode/
│   └── foxglove_protobuf_encode.hpp         # SerializeFoxglove / SerializeLog …
├── schema/
│   ├── foxglove_channel_schema.hpp / .cpp   # SchemaConverter
│   └── protobuf_ros_style_name.hpp / .cpp   # RosStyleTypeName
├── message/
│   ├── convert_automsgs_message.hpp / .cpp   # ConvertMessageToFoxglove
│   ├── convert_automsgs_body_3d.hpp / .cpp # TryConvertBodyToFoxglove
│   └── convert_nav_path_to_scene.hpp / .cpp # PathToSceneUpdate
└── tools/
    └── generate_automsgs_foxglove_registry.py
```

## 调用关系

1. **`registry`**：`FindFoxgloveRule`（有序表 + `lower_bound`）、`AutomsgsFoxgloveStrategy`；数据在生成的 `*_data.hpp`（池化字符串 + `static_assert` 有序）。
2. **`schema`**：`SchemaConverter::AddFromFullName` / `ListSchemas`；`RosStyleTypeName` 供展示名。
3. **`convert_automsgs_message`**：总调度 → **`TryConvertBodyToFoxglove`** 或 **`BuildFallbackLog`**（实现内静态）。
4. **`encode`**：`SerializeFoxglove`、`SerializeSceneUpdate`、`SerializeLog`。

## 重新生成 `automsgs_foxglove_registry_data.hpp`

```bash
python3 src/autonomy/autoviz/core/convert/tools/generate_automsgs_foxglove_registry.py
```

## 扩展新消息类型

1. 编辑 `tools/generate_automsgs_foxglove_registry.py` 的 `SPECIAL`，再运行脚本。
2. 在 **`message/convert_automsgs_body_3d.cpp`** 的 `TryConvertBodyToFoxglove` 中增加分支。

## 测试

`-DAUTOVIZ_BUILD_TEST=ON` 且系统有 **GTest** 时，构建 `autoviz_registry_test`，覆盖 `FindFoxgloveRule`（见仓库 `autoviz/README.md`）。

## 参考

- `src/foxglove-sdk/schemas/README.md`
- `../README.md`
