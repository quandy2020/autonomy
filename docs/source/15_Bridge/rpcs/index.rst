automsgs.rpcs API
=================

系统测试、集成开发、算法联调（经 gRPC 驱动机载任务）。

- **§3 总览**：:doc:`协议与 Proto <../03_rpc_protocol>`
- **本目录**：``01_*``–``12_*``；Command 页（``07``–``11``）说明各域服务字段与 grpcurl / ``rpc-cli.py`` 示例

**阅读顺序**：:doc:`01 接入与验证 <01_connection_guide>` → :doc:`02 服务概述 <02_service_overview>` → :doc:`03 公共类型 <03_common_types>` → Query / Stream / System → Command → :doc:`12 集成测试 <12_integration_tests>`

契约校验：``python3 docs/scripts/check_bridge_rpc_docs.py``

.. toctree::
   :maxdepth: 1

   总览 <../03_rpc_protocol>
   01 接入与验证 <01_connection_guide>
   02 服务概述 <02_service_overview>
   03 公共消息类型 <03_common_types>
   04 查询接口 Query <04_query_api>
   05 推送接口 Stream <05_stream_api>
   06 系统控制 System <06_system_api>
   07 NavigationService <07_navigation_command>
   08 FollowService <08_follow_command>
   09 TeleopService <09_teleop_command>
   10 ChargeService <10_dock_command>
   11 MapService <11_map_command>
   12 集成测试参考 <12_integration_tests>
