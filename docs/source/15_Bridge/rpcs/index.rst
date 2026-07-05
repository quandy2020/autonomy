AutonomyService API
===================

**适用读者**：系统测试、集成开发、算法联调（通过 gRPC 驱动机载任务）。

- **§3 总览**：:doc:`协议与 Proto <../03_rpc_protocol>`
- **本目录**：``01_*``–``13_*``；Command 页（``07``–``12``）含 Proto 原文、字段说明与 grpcurl 示例

**阅读顺序**：:doc:`01 接入与验证 <01_connection_guide>` → :doc:`02 服务概述 <02_service_overview>` + :doc:`03 公共消息类型 <03_common_types>` → :doc:`04 查询接口 <04_query_api>` … :doc:`12 地图 Command <12_map_command>` → :doc:`13 集成测试参考 <13_integration_tests>`

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
   07 SendNavigationCommand <07_navigation_command>
   08 SendExplorationCommand <08_exploration_command>
   09 SendFollowCommand <09_follow_command>
   10 SendTeleopCommand <10_teleop_command>
   11 SendDockCommand <11_dock_command>
   12 SendMapCommand <12_map_command>
   13 集成测试参考 <13_integration_tests>
