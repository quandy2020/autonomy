# bag_convert

将 ROS1 `.bag` 转为 autolink `.record`，供 `autolink_recorder play` 回放。

## 依赖

```bash
python -m pip install -r autonomy/tools/bag_convert/requirements.txt
```

## 用法

```bash
python -m autonomy.tools.bag_convert input.bag -o ./data/records --backpack-2d
python -m autonomy.tools.bag_convert --list-types
```

预设见 `presets/backpack_2d.json`。

## 模块（一文件一类）

| 文件 | 类 | 公开方法 |
|------|-----|----------|
| `bag_convert_cli.py` | `BagConvertCli` | `run_main` |
| `bag_converter.py` | `BagConverter` | `convert_bag` |
| `ros_converter.py` | `RosConverter` | `convert_ros_message` |
| `proto_registry.py` | `ProtoRegistry` | `setup_proto_import_path`, `list_supported_ros_types`, `resolve_proto_class` |
| `record_writer.py` | `RecordWriter` | `open_record`, `write_channel`, `write_message`, `close_record`, `encode_proto_desc` |
| `bag_convert_config.py` | `BagConvertConfig` | `create_default` |
| `dependency_checker.py` | `DependencyChecker` | `check_runtime_dependencies` |
| `preset_store.py` | `PresetStore` | `load_preset` |

## Python API

```python
from pathlib import Path
from autonomy.tools.bag_convert import BagConverter, PresetStore

topics, topic_remap = PresetStore().load_preset("backpack_2d")
total, counts = BagConverter().convert_bag(
    Path("input.bag"), Path("output.record"), False, topics, topic_remap,
)
```
