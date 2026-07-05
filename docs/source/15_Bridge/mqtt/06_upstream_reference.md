(mqtt-upstream)=
# 上游参考

> [§5 MQTT 总览](../05_mqtt.md) · 本专题 **mqtt/06**（H2 **6.x**）。

Bridge MQTT 文档以 **[Eclipse Mosquitto](https://mosquitto.org/)** 为 Broker 与客户端库参考实现。下列链接来自 [官方 GitHub README](https://github.com/eclipse-mosquitto/mosquitto#links) 与 [mosquitto.org](https://mosquitto.org/)。

## 6.1 官方资源

| 资源 | URL | 用途 |
|------|-----|------|
| 项目主页 | [mosquitto.org](https://mosquitto.org/) | 下载、文档入口 |
| 源码仓库 | [github.com/eclipse-mosquitto/mosquitto](https://github.com/eclipse-mosquitto/mosquitto) | Issue、Release、示例 |
| 二进制下载 | [mosquitto.org/download](https://mosquitto.org/download/) | 各平台安装包 |
| Man pages | [mosquitto.org/man](https://mosquitto.org/man/) | `mosquitto.conf`、`mosquitto_pub/sub` |
| C API | [mosquitto.org/api](https://mosquitto.org/api/) | `libmosquitto` 函数参考 |
| 公共测试 Broker | [test.mosquitto.org](https://test.mosquitto.org/) | 开发联调（非生产） |
| 安全公告 | [mosquitto.org/security](https://mosquitto.org/security/) | CVE 与升级 |

## 6.2 MQTT 标准

| 版本 | 规范 |
|------|------|
| MQTT 3.1.1 | [OASIS MQTT v3.1.1](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/mqtt-v3.1.1.html) |
| MQTT 5.0 | [OASIS MQTT v5.0](https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.html) |
| 社区 | [mqtt.org](http://mqtt.org/) |

Mosquitto Broker 同时支持 5.0 / 3.1.1 / 3.1；Bridge 首版建议 **3.1.1**（生态最广），后续可选 5.0 用户属性携带 `cmd_id`。

## 6.3 常用命令速查

```bash
# Broker
mosquitto -c /etc/mosquitto/mosquitto.conf

# 用户
mosquitto_passwd -c /path/to/passwd username

# 订阅 / 发布
mosquitto_sub  -h HOST -p PORT -u USER -P PASS -t 'topic' -v
mosquitto_pub  -h HOST -p PORT -u USER -P PASS -t 'topic' -m 'payload' -q 1

# 动态安全（MQTT 5，可选）
mosquitto_ctrl dynsec ...
```

## 6.4 仓库结构（与 Bridge 相关部分）

摘自 [eclipse-mosquitto/mosquitto](https://github.com/eclipse-mosquitto/mosquitto) 目录：

| 路径 | 内容 |
|------|------|
| `src/` | Broker 主程序 |
| `lib/` | **libmosquitto** 客户端库（`MqttBridge` 依赖） |
| `client/` | `mosquitto_pub` / `mosquitto_sub` / `mosquitto_rr` |
| `mosquitto.conf` | 默认配置模板 |
| `man/` | man page 源 |
| `examples/` | 客户端示例代码 |
| `docker/` | 容器镜像 |

## 6.5 其他 MQTT 客户端库

| 库 | 说明 |
|----|------|
| [Eclipse Paho](https://www.eclipse.org/paho/) | 多语言客户端；Autonomy C++ 优先 **libmosquitto** 与 Mosquitto 工具链一致 |
| Python `paho-mqtt` | 脚本、集成测试 |
| Node.js `mqtt.js` | Web / 移动端原型 |

Broker 可替换为 EMQX、HiveMQ、AWS IoT Core 等，只要 Topic 与 [mqtt/04 Topic 协议](04_topic_protocol.md) 一致。

---

**导航**：[← 05 Bridge 集成](05_bridge_integration.md) · [§5 总览 →](../05_mqtt.md)
