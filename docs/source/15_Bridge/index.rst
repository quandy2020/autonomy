Bridge 通信桥接
=================

``autonomy/bridge`` 是 Autonomy 的**外部通信桥接**子系统，向云/移动端/第三方暴露 gRPC 与 MQTT 接口。

**§0–§6**：指南 · 概览 · 架构 · gRPC / MQTT · RPC 协议 · 综述。

**阅读路径** — 新手：§0 → §2 → §3；集成 / 调 API：§5 ``rpcs/`` → §3；协议：§6 → §3 / §4。

.. toctree::
   :maxdepth: 2

   0 指南 <00_guide>
   1 模块概览 <01_overview>
   2 模块架构设计 <02_architecture>
   3 gRPC 桥接插件 <03_grpc>
   4 MQTT 桥接插件 <04_mqtt>
   5 RPCs 协议 <rpcs/index>
   6 通信协议综述 <06_survey>
