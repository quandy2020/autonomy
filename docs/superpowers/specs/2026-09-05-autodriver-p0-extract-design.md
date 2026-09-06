# autodriver P0 提取优化 Design

**Date:** 2026-09-05  
**Status:** Approved (chat)  
**Scope:** 从 Apollo drivers 内化六项；不引入 Cyber DAG

## Items

1. Lidar 三段接口 + 可选 Scan 发布  
2. PoseLookup 接线（补偿可实车启用路径）  
3. Stream：TCP + NTRIP  
4. Velodyne beam 标定 YAML + model 分支  
5. Hesai 骨架 + Registry  
6. CAN Receiver + MessageManager 生产化骨架  

## Non-goals

Cyber/DAG、真 Hesai 几何、NTRIP 外网联调、V4L2、毫米波雷达、主动 commit

## Success

各有单测或可编译接线；YAML/文档更新；对齐计划勾选对应项
