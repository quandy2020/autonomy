# Log schema

glog 前缀：`[ts_utc][ts_mono][trace_id][file:line]`。

crash dump → `robot/log/`，`log_rotate.sh` 不删未上传 P0。热路径禁止 `LOG(INFO)` 打印通道 payload。从 ERROR 反查见 [evidence_chain](evidence_chain.md)。
