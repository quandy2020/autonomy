Bridge 通信桥接
=================

``autonomy/bridge`` 是 Autonomy 的**外部通信桥接**子系统，向云/移动端/第三方暴露 gRPC 与 MQTT 接口。文档按 **§0–§6** 排列（**§0** 指南含概览；**§1** 为 §0.3 配置详表）；**§3–§5** 各含总览页 + 子目录专题。机载 ``bridge.lua`` 字段见 [§1 参数配置](01_options.md)（从 [§0.3 配置入口](00_guide.md#03-配置入口) 进入，不列入下方目录）。

.. toctree::
   :maxdepth: 2
   :titlesonly:

   0 指南 <00_guide>
   2 架构设计 <02_architecture>
   3 RPC 协议 <rpcs/index>
   4 gRPC <grpc/index>
   5 MQTT <mqtt/index>
   6 协议综述 <06_survey>
