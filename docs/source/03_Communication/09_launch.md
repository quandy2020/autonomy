# 9. 启动 Launch

车上多进程靠 **`.launch`** 编排；单进程里用 **mainboard + `.dag`** 拉 Component。Autonomy 的 `autonomy.launch` / `task.launch` 都是这一层。

`.launch` 描述多进程/多 DAG 编排，`.dag`+mainboard 拉 Component；`autonomy.launch` 与精简 `task.launch` 差别在启哪些 Server。改 launch 前先确认 `AUTOLINK_PATH` 与 working dir，否则找不到 `.so` 或 conf。

## DAG

```text
module_config {
    module_library : "…/libcommon_component_example.so"
    components {
        class_name : "CommonComponentSample"
        config {
            name : "common"
            readers { channel: "/autolink/prediction" }
            readers { channel: "/autolink/test" }
        }
    }
}
```

TimerComponent：`timer_components` + `interval`（ms），无 `readers`。

| 字段 | 含义 |
|------|------|
| `module_library` | `.so`（`AUTOLINK_LIB_PATH`） |
| `class_name` | 与 `AUTOLINK_REGISTER_COMPONENT` 一致 |
| `config.name` | 实例名 → 内部 Node 名 |
| `readers[].channel` | 输入 channel |
| `interval` | Timer 周期（ms） |

## Launch XML

```xml
<autolink>
<module>
<name>common</name>
<dag_conf>…/common.dag</dag_conf>
<process_name>common</process_name>
</module>
</autolink>
```

```bash
export AUTOLINK_PATH=<prefix>
mainboard -d …/common.dag
autolink_launch …/common.launch
mainboard -d foo.dag --plugin=/path/to/plugins.xml
```

| 变量 | 用途 |
|------|------|
| `AUTOLINK_DAG_PATH` | `.dag` |
| `AUTOLINK_LIB_PATH` | Component `.so` |
| `AUTOLINK_LAUNCH_PATH` | `.launch` |

Autonomy 全栈：`autonomy/system/launch/autonomy.launch`，见 [04 Running](../04_Running/03_autonomy_process.md)。
