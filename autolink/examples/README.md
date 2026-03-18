# Autolink 示例

## talker / listener（发布-订阅）

- **talker**：向 `channel/chatter` 发送 `Chatter` 消息。
- **listener**：订阅 `channel/chatter`，收到后打印 seq 与 content。

### 运行方式

1. **配置路径**（二选一）  
   - 安装后使用：`export AUTOLINK_PATH=/usr/local/share/autolink`（确保该目录下有 `conf/autolink.pb.conf`）。  
   - 源码/build 下运行：`export AUTOLINK_PATH=/path/to/autolink`（该路径下要有 `conf/autolink.pb.conf`）。

2. **先起 listener，再起 talker**（推荐）  
   ```bash
   # 终端 1
   ./autolink_example_listener

   # 终端 2（同机、同 AUTOLINK_PATH）
   ./autolink_example_talker
   ```  
   listener 会通过发现机制收到 talker，并打印 `Received message seq-> ...`、`msgcontent-> ...`。

3. **先起 talker 再起 listener**  
   也可以先运行 talker，再运行 listener；listener 加入拓扑后会收到后续消息。

### 若收不到数据（含 SHM 收不到）

- 确认 **talker 和 listener 使用同一套配置**（同一 `AUTOLINK_PATH`，且能读到 `conf/autolink.pb.conf`）。
- **同机两进程**：走 SHM。listener 日志里应出现 **「SHM receiver enabled for channel [channel/chatter] from writer (discovery ok)」**；若没有，说明两端尚未通过 RTPS 发现，writer 未在 listener 侧 Enable，SHM 不会收消息。
- **发现依赖 RTPS**：即使数据走 SHM，发现仍靠 RTPS。需保证 RTPS participant 正常（配置里 `participant_attr` 的 `lease_duration`/`announcement_period` 已配好）、两端 `AUTOLINK_DOMAIN_ID` 一致（默认 80），同机多进程一般即可发现。
- **跨机/跨容器**：需 RTPS 发现互通，可设 `export AUTOLINK_DOMAIN_ID=80`（两端一致），并保证网络/组播可达；必要时设 `AUTOLINK_IP` 为本机 IP。
