(mqtt-broker)=
# Broker 部署

> [§5 MQTT 总览](../05_mqtt.md) · 本专题 **mqtt/02**（H2 **2.x**）。

本章说明如何部署 [Eclipse Mosquitto](https://mosquitto.org/) Broker，供 Bridge `MqttBridge` 与场外客户端连接。内容对齐官方 [Quick start](https://github.com/eclipse-mosquitto/mosquitto#quick-start) 与 [mosquitto.conf(5)](https://mosquitto.org/man/mosquitto-conf-5.html)。

## 2.1 安装

| 平台 | 方式 |
|------|------|
| Linux (Debian/Ubuntu) | `sudo apt install mosquitto mosquitto-clients` |
| macOS | `brew install mosquitto` |
| Docker | 官方镜像见 [mosquitto/docker](https://github.com/eclipse-mosquitto/mosquitto/tree/master/docker) |
| 源码 | [mosquitto.org/download](https://mosquitto.org/download/) 或 `git clone` [eclipse-mosquitto/mosquitto](https://github.com/eclipse-mosquitto/mosquitto) |

安装后通常包含：`mosquitto`（Broker）、`mosquitto_pub` / `mosquitto_sub`（CLI）、`libmosquitto`（开发库）。

## 2.2 快速验证（本机）

官方最小流程（**仅本机匿名**，适合初次测试）：

```bash
# 终端 1：启动 Broker（无配置文件，仅 localhost）
mosquitto

# 终端 2：订阅
mosquitto_sub -t 'test/topic' -v

# 终端 3：发布
mosquitto_pub -t 'test/topic' -m 'hello world'
```

> 无 `-c` 配置启动时，Mosquitto 仅允许**本机**匿名连接。若需局域网或公网客户端接入，**必须**提供配置文件并配置认证，见 [§2.3](#23-autonomy-bridge-用-mosquittoconf)。

公共测试服务器：[test.mosquitto.org](https://test.mosquitto.org/)（支持 plain / TLS / WebSocket；**勿用于生产或敏感数据**）。

## 2.3 Autonomy Bridge 用 `mosquitto.conf`

Bridge 开发联调推荐配置（局域网 Broker，用户名密码 + Topic ACL）：

```ini
# /etc/mosquitto/conf.d/autonomy.conf

# 监听 plain MQTT（开发）；生产请改用 8883 + TLS，见 §2.5
listener 1883
protocol mqtt

# 禁止匿名（公网或局域网均需认证）
allow_anonymous false
password_file /etc/mosquitto/autonomy_passwd

# 持久化（可选，Broker 重启保留会话）
persistence true
persistence_location /var/lib/mosquitto/

# 日志
log_dest file /var/log/mosquitto/mosquitto.log
log_type error
log_type warning
log_type notice
log_type information

# ACL：见 §2.4
acl_file /etc/mosquitto/autonomy_acl
```

创建用户（`MqttBridge` 与移动端各一账号）：

```bash
sudo mosquitto_passwd -c /etc/mosquitto/autonomy_passwd bridge_robot_01
sudo mosquitto_passwd    /etc/mosquitto/autonomy_passwd mobile_app_01
```

启动：

```bash
mosquitto -c /etc/mosquitto/mosquitto.conf
```

## 2.4 Topic ACL 示例

对齐 [mqtt/04 Topic 协议](04_topic_protocol.md) 命名，限制各客户端读写范围（语法见 [mosquitto.conf acl](https://mosquitto.org/man/mosquitto-conf-5.html)）：

```ini
# /etc/mosquitto/autonomy_acl
# cmd/# 覆盖 navigation / exploration / follow / teleop / dock / map / emergency_stop

# MqttBridge（机载）：读 cmd + query，写 state/event/ack
user bridge_robot_01
topic read  autonomy/robot_01/cmd/#
topic read  autonomy/robot_01/query/#
topic write autonomy/robot_01/state/#
topic write autonomy/robot_01/event
topic write autonomy/robot_01/ack/#

# 移动端：写 cmd/query，读 state/event/ack
user mobile_app_01
topic write autonomy/robot_01/cmd/#
topic write autonomy/robot_01/query/#
topic read  autonomy/robot_01/state/#
topic read  autonomy/robot_01/event
topic read  autonomy/robot_01/ack/#
```

`{robot_id}` 与 `bridge.lua` / 环境变量一致；多机群为每台机器人独立 ACL 段。

## 2.5 TLS（MQTTS）

生产环境应对外提供 **8883** 端口并启用 TLS（OpenSSL）。Mosquitto 支持：

- 服务端证书 + 密钥
- 可选客户端证书（双向 TLS）
- Let's Encrypt 集成见上游 [README-letsencrypt.md](https://github.com/eclipse-mosquitto/mosquitto/blob/master/README-letsencrypt.md)

```ini
listener 8883
protocol mqtt
cafile   /etc/mosquitto/certs/ca.crt
certfile /etc/mosquitto/certs/server.crt
keyfile  /etc/mosquitto/certs/server.key
require_certificate false   # true = 双向 TLS
```

`MqttBridge` 侧对应 `MqttOptions` 扩展字段（TLS CA、client cert，待 proto 补充）。

## 2.6 与 `bridge.lua` 端口对齐

| 配置 | 当前 `bridge.lua` | Mosquitto 默认 | 建议 |
|------|-------------------|----------------|------|
| MQTT 端口 | `12345` | `1883` | 联调时统一为 `1883`，或 Broker `listener` 与 lua `port` 一致 |

```lua
-- config/bridge/bridge.lua
mqtt = {
    host = "127.0.0.1",   -- Broker 地址（非 Bridge 监听地址）
    port = 1883,
}
```

> **注意**：MQTT 模式下 Bridge 是 **Client**，主动 `CONNECT` 到 Broker；不存在「Bridge 监听 MQTT 端口」—— 与 gRPC `GrpcBridgeServer` 监听 `host:port` 不同。

## 2.7 联调 Autonomy Topic

Broker 就绪后，用 CLI 模拟场外客户端（Topic 见 [mqtt/04](04_topic_protocol.md)）：

```bash
# 订阅状态（模拟 App）
mosquitto_sub -h 127.0.0.1 -p 1883 -u mobile_app_01 -P '<password>' \
  -t 'autonomy/robot_01/state/robot' -v

# 发布导航指令（模拟 App）
mosquitto_pub -h 127.0.0.1 -p 1883 -u mobile_app_01 -P '<password>' \
  -t 'autonomy/robot_01/cmd/navigation' -q 1 \
  -m '{"command":"NAV_CMD_STOP","timestamp":{"sec":0,"nanosec":0}}'
```

`MqttBridge` 实现后，机载进程将使用 `bridge_robot_01` 账号订阅 `cmd/#` 并发布 `state/`。

---

**导航**：[← 01 Mosquitto](01_overview.md) · [03 libmosquitto →](03_libmosquitto_api.md)
