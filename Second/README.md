# 仿真方案2总结

## 仿真参数表


| **参数类别**   | **参数名称**   | **参数值/配置**                                    | **说明**                              |
| -------------- | -------------- | -------------------------------------------------- | ------------------------------------- |
| **基础参数**   | 节点数量       | `numNodes = 20`                                    | 无人机集群规模                        |
|                | 仿真时间       | `simulationTime = 60.0`                            | 总仿真时长（秒）                      |
|                | 仿真区域大小   | `SIM_AREA_SIZE = 500.0`                            | 三维区域范围：X/Y 0~500米，Z 50~150米 |
| **移动模型**   | 移动模型类型   | `GaussMarkovMobilityModel`                         | 高斯-马尔可夫模型（平滑随机移动）     |
|                | 平均速度范围   | `UniformRandomVariable[Min=10                      | Max=20]` m/s                          |
|                | 移动区域限制   | `Box(0, 500, 0, 500, 50, 150)`                     | 节点不可超出该三维区域                |
| **通信时隙**   | 拓扑更新周期   | `TOPOLOGY_UPDATE_INTERVAL = 5.0` 秒                | 每隔 5 秒检测节点位置并更新通信链路   |
|                | 数据包发送间隔 | `PACKET_INTERVAL = 0.5` 秒                         | 每个活动链路每 0.5 秒发送一次数据包   |
| **物理层参数** | 通信有效范围   | `COMM_RANGE = 250.0` 米                            | 节点间距离 ≤250 米时建立链路         |
|                | 发射功率       | `TxPowerStart = 23.0 dBm`, `TxPowerEnd = 23.0 dBm` | 固定功率                              |
|                | 接收灵敏度     | `RxSensitivity = -85.0 dBm`                        | 最小接收信号强度                      |
| **MAC层参数**  | MAC 类型       | `AdhocWifiMac`                                     | 支持 QoS 的 Ad-Hoc 模式               |
|                | QoS 支持       | `QosSupported = true`                              | 启用服务质量保障                      |
| **网络层参数** | 路由协议       | `AODV`                                             | 按需路由协议，动态维护拓扑            |
| **应用层参数** | 流量类型       | UDP 双向通信（请求-ACK 机制）                      | 数据包与 ACK 独立记录                 |
|                | 数据包大小     | `PacketSize = 512 Bytes`                           | 固定包大小                            |
|                | 服务器端口     | 2000                                               | 固定端口监听 ACK 请求                 |

---

## 协议栈分析表


| **协议层**     | **协议/配置**                      | **关键特性**                                   |
| -------------- | ---------------------------------- | ---------------------------------------------- |
| **应用层**     | `UdpEchoClient` / `UdpEchoServer`  | 实现请求-ACK 机制，双向通信记录                |
| **传输层**     | UDP                                | 无连接、低延迟，适用于实时通信                 |
| **网络层**     | AODV 路由协议                      | 动态维护路由表，适应拓扑变化                   |
| **数据链路层** | `AdhocWifiMac` (802.11ac)          | 支持 QoS 的分布式协调功能（DCF），优化多跳通信 |
| **物理层**     | OFDM (802.11ac) +`YansWifiChannel` | 高频段、高吞吐量，默认传播模型                 |

---

## 关键时隙与空间行为表


| **行为类型**       | **触发机制**                                                             | **时间/空间特性**                                                |
| ------------------ | ------------------------------------------------------------------------ | ---------------------------------------------------------------- |
| **拓扑更新时隙**   | 周期性调用`UpdateTopology()`                                             | 每 5 秒检测节点位置，基于距离 (`COMM_RANGE`) 更新活动链路        |
| **数据包发送时隙** | `ScheduleTransmissions()` 动态调度                                       | 每 0.5 秒为活动链路创建双向 UDP 客户端                           |
| **ACK 记录时隙**   | 服务器接收数据包 (`ServerReceive`) 和客户端接收 ACK (`ClientReceiveAck`) | 实时记录 ACK 发送与接收事件                                      |
| **空间移动行为**   | 三维高斯-马尔可夫模型                                                    | X/Y/Z 轴独立随机运动，速度动态变化，区域约束（500×500×100 米） |

---

## 关键代码逻辑说明

### 1. 动态拓扑更新

```cpp
void UpdateTopology(NodeContainer& nodes) {
    activeLinks.clear();
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        for (uint32_t j = i+1; j < nodes.GetN(); ++j) {
            double distance = CalculateDistance(mob[i]->GetPosition(), mob[j]->GetPosition());
            if (distance <= COMM_RANGE) {
                activeLinks[{i,j}] = true; // 双向链路
            }
        }
    }
}
```

### 2. 数据包传输机制

```cpp
void ScheduleTransmissions() {
    for (auto& link : activeLinks) {
        if(link.second) {
            CreateClientApplication(link.first.first, link.first.second); // 双向通信
            CreateClientApplication(link.first.second, link.first.first);
        }
    }
    Simulator::Schedule(Seconds(PACKET_INTERVAL), &ScheduleTransmissions);
}
```
