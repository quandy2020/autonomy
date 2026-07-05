(mqtt-libmosquitto)=
# libmosquitto

> [§5 MQTT 总览](../05_mqtt.md) · 本专题 **mqtt/03**（H2 **3.x**）。

`MqttBridge` 规划使用 Mosquitto 自带的 **C 客户端库 [libmosquitto](https://mosquitto.org/api/)**（非 Paho）。API 文档：[mosquitto.org/api](https://mosquitto.org/api/)；man page：[mosquitto(7)](https://mosquitto.org/man/mosquitto-7.html)、[mosquitto_sub(1)](https://mosquitto.org/man/mosquitto-sub-1.html) 行为与库调用一致。

## 3.1 核心概念

| 概念 | 说明 |
|------|------|
| `struct mosquitto` | 客户端实例；每连接一个 |
| `mosquitto_new()` | 创建实例，指定 `client_id`、clean session |
| `mosquitto_connect()` | TCP 连接 Broker（`host`, `port`, `keepalive`） |
| `mosquitto_loop()` | 驱动网络 I/O 与回调（单线程） |
| `mosquitto_loop_start()` | 后台线程跑 loop（需 **pthreads**） |
| 回调 | `connect` / `message` / `subscribe` / `disconnect` 等 |

Bridge 推荐 **`mosquitto_loop_start()` + 业务线程**：与 gRPC Event 线程模型类似，避免阻塞导航主循环。

## 3.2 最小订阅示例（C）

```c
#include <mosquitto.h>
#include <stdio.h>
#include <string.h>

static void on_message(struct mosquitto *mosq, void *obj,
                       const struct mosquitto_message *msg) {
    printf("topic=%.*s payload=%.*s\n",
           msg->topiclen, msg->topic,
           msg->payloadlen, (char *)msg->payload);
}

int main(void) {
    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new("autonomy_bridge_01", true, NULL);
    mosquitto_message_callback_set(mosq, on_message);

    mosquitto_username_pw_set(mosq, "bridge_robot_01", "secret");
    mosquitto_connect(mosq, "127.0.0.1", 1883, 60);
    mosquitto_subscribe(mosq, NULL, "autonomy/robot_01/cmd/#", 1);

    mosquitto_loop_forever(mosq, -1, 1);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
```

编译（系统已安装 dev 包）：

```bash
gcc -o mqtt_sub mqtt_sub.c -lmosquitto
```

## 3.3 发布与 QoS

```c
// QoS 1，导航 ack
mosquitto_publish(mosq, NULL,
    "autonomy/robot_01/ack/cmd-uuid",
    payload_len, payload_json, 1, false);

// QoS 0，高频状态
mosquitto_publish(mosq, NULL,
    "autonomy/robot_01/state/robot",
    state_len, state_bytes, 0, false);
```

| QoS | Mosquitto API | Bridge 用途 |
|-----|---------------|-------------|
| 0 | `qos=0` | `state/robot` 高频位姿 |
| 1 | `qos=1` | `cmd/*`、`event`、`ack/*` |
| 2 | `qos=2` | 一般不用（开销大） |

## 3.4 TLS 连接

```c
mosquitto_tls_set(mosq, ca_path, NULL, cert_path, key_path, NULL);
mosquitto_tls_insecure_set(mosq, false);
mosquitto_connect(mosq, broker_host, 8883, 60);
```

对应 Broker [mqtt/02 §2.5](02_mosquitto_broker.md#25-tlsmqtts) 配置。

## 3.5 C++ 封装建议（MqttBridge）

`plugins/mqtt/mqtt_bridge.hpp` 规划接口：

```cpp
class MqttBridge {
public:
    explicit MqttBridge(const proto::MqttOptions& options);
    void Start();    // connect + subscribe + loop_start
    void Stop();     // loop_stop + disconnect

private:
    static void OnMessage(struct mosquitto*, void* userdata,
                          const struct mosquitto_message* msg);
    void HandleNavigationCmd(const std::string& payload);
    void PublishState(const commmsgs::proto::vehicle_msgs::RobotState& state);

    struct mosquitto* mosq_{nullptr};
    proto::MqttOptions options_;
    std::string robot_id_;
};
```

| 设计点 | 建议 |
|--------|------|
| 线程模型 | `mosquitto_loop_start`；`OnMessage` 内仅解析 + 投递队列，FSM 在 Worker 线程 |
| 重连 | `mosquitto_reconnect_async` 或 connect 回调中指数退避 |
| 载荷 | P0 JSON（`nlohmann` / `common::json_util`）；P1 Protobuf bytes |
| 与 gRPC 共享 | 导航 FSM 抽至 `bridge/common/command_fsm.hpp`，两插件共用 |

## 3.6 与 CLI 工具对照

实现前可用 CLI 验证 Broker 与 Topic（见 [mqtt/02 §2.7](02_mosquitto_broker.md#27-联调-autonomy-topic)）：

| libmosquitto API | CLI 等价 |
|------------------|----------|
| `mosquitto_sub` | `mosquitto_sub -t 'topic' -v` |
| `mosquitto_publish` | `mosquitto_pub -t 'topic' -m 'payload' -q 1` |
| `mosquitto_rr` | 请求-响应模式（Bridge 可不使用） |

## 3.7 构建依赖（规划）

| 依赖 | 说明 |
|------|------|
| `libmosquitto` | `-lmosquitto`；Debian 包 `libmosquitto-dev` |
| OpenSSL | TLS 时需要（`WITH_TLS`） |
| pthreads | `mosquitto_loop_start()` 必需 |

Bazel/CMake 尚未引入；可参考 Mosquitto [README-compiling.md](https://github.com/eclipse-mosquitto/mosquitto/blob/master/README-compiling.md) 或系统包。

---

**导航**：[← 02 Broker 部署](02_mosquitto_broker.md) · [04 Topic 协议 →](04_topic_protocol.md)
