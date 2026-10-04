# 使用

## 启动

```bash
./build/bin/autoviz
./build/bin/autoviz --config config/default.autoviz
./build/bin/autoviz file.record          # 打开并回放 Autolink 记录
```

常用参数：

| 参数 | 说明 |
|------|------|
| `-c` / `--config` | 加载 `.autoviz` 会话 |
| `-s` / `--splash-screen` | 启动图路径；空路径关闭 splash |
| 位置参数 | `.autoviz` 与/或 `.record` / `.bag` / `.mcap` |

配合 fakedata：

```bash
# 终端 1
./bin/autonomy_foxglove_fakedata
# 终端 2
./build/bin/autoviz --config config/default.autoviz
```

## 环境变量

| 变量 | 分隔符 | 说明 |
|------|--------|------|
| `AUTOVIZ_PLUGIN_PATH` | `:` / `;` | Display / Tool / View / Transformer 插件目录 |
| `AUTOVIZ_RESOURCE_PATH` | 同上 | `package://` mesh 搜索前缀 |
| `AUTOVIZ_OGRE_MEDIA_PATH` | — | Ogre 资源根（可选覆盖） |
| `AUTOVIZ_OGRE_PLUGIN_DIR` | — | Ogre 插件目录（可选） |

```bash
export AUTOVIZ_RESOURCE_PATH=/opt/autonomy/share:/home/user/robot_assets
export AUTOVIZ_PLUGIN_PATH=/opt/autonomy/lib/autoviz_plugins
./bin/autoviz --config share/autonomy/autoviz/default.autoviz
```

## 会话配置

| 格式 | 说明 |
|------|------|
| **`.autoviz`** | 原生会话（推荐）：Display / View / Tool / Panel |
| **`.rviz`** | 只读导入；类名映射到 Autoviz，不依赖 ROS |

会话由 `SessionConfig` + YAML 读写；Displays 面板属性树可保存 / 另存。

## 回放与导入

| 源 | 行为 |
|----|------|
| `.record` | 直接打开（File → Open Record、拖放、CLI） |
| `.bag` | 查找 PATH 中 `bag_to_record`（兼容 `rosbag_to_record`）后转换 |
| `.mcap` | 查找 PATH 中 `mcap_to_record`；未安装时提示离线转换 |

## UI 结构（摘要）

| 区域 | 职责 |
|------|------|
| Displays | Display 树、属性、Channel 选择 |
| Views | 视口 / ViewController |
| Tools | Interact、导航目标、测距等 |
| Time / Playback | `.record` 播放控制 |
| Selection | 3D 拾取结果 |
| Image dock | Image Display 弹出 / 停靠 |

## 相关文档

- [构建](build.md) · [架构模块](../architecture/modules.md)
