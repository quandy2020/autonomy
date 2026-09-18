# bag_convert

将 ROS1 `.bag` 转为 autolink `.record`，供 `autolink recorder play` 回放。

## 依赖

```bash
python -m pip install -r autonomy/tools/bag_convert/requirements.txt
```

## 用法

```bash
python -m autonomy.tools.bag_convert input.bag -o ./data/records --backpack-2d
python -m autonomy.tools.bag_convert --list-types
python -m autonomy.tools.bag_convert --info input.bag
python -m autonomy.tools.bag_convert /data/mv/campus_train0_04.bag -o ./data/records --vbr
python -m autonomy.tools.bag_convert /data/mv/campus_train0_04.bag -o ./data/records --vbr --duration-sec 60
```

在仓库根（`src/autonomy`）下执行，保证 `python -m autonomy.tools.bag_convert` 可导入。

预设见 `presets/`:

| 预设 | 用途 |
|------|------|
| `backpack_2d` | 2D 激光 + IMU |
| `vbr` | VBR / MV 数据集：`/ouster/points` + `/imu/data`（保留 leading `/`） |

## 模块（一文件一类）

| 文件 | 类 | 公开方法 |
|------|-----|----------|
| `bag_convert_cli.py` | `BagConvertCli` | `run_main` |
| `bag_converter.py` | `BagConverter` | `inspect_bag`, `convert_bag` |
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

preset = PresetStore().load_preset("vbr")
total, counts = BagConverter().convert_bag(
    Path("input.bag"), Path("output.record"),
    use_leading_slash=bool(preset.leading_slash),
    topic_filter=preset.topics,
    topic_remap=preset.topic_remap,
    skip_unsupported=bool(preset.skip_unsupported),
)
```
