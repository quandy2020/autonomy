(communication-guide)=
# 0. Communication 通信框架指南

`autolink` 是 Autonomy 的进程间通信运行时，角色接近 Cyber RT / ROS 2 那一层 DDS。架构见 [01](01_architecture.md)；Node / Channel / Service / Action / Parameter 在 [02](02_node.md)–[06](06_parameter.md)；Plugin、Log 在 [07](07_plugin.md)、[08](08_log.md)；Launch、Component、Timer、Scheduler 在 [09](09_launch.md)–[12](12_scheduler.md)。

先 `Init()`；`node_name` 要唯一；channel 名和消息类型两端一致；回调里别堵；同机多进程共享同一份 `AUTOLINK_PATH`。

```bash
source autolink/autolink/setup.bash
export AUTOLINK_PATH=<含 conf/autolink.pb.conf 的目录>
```

示例在 `autolink/examples/cpp/` 和 `autolink/examples/python/`。

(communication-usage)=
