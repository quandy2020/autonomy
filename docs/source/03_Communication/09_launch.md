# 9. 启动 Launch

**mainboard**：单进程加载 DAG、拉起 Component。 
**autolink_launch**：解析 `.launch`、编排多进程。

## 9.1 DAG

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

## 9.2 Launch XML

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
