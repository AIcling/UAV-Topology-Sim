#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/vector.h"
#include "ns3/flow-monitor-module.h"
#include <map>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavAdhocNetwork");

const double SIM_AREA_SIZE = 500.0;     // 仿真区域大小（米）
// const double UAV_SPEED = 15.0;          // 平均移动速度（m/s）
const double COMM_RANGE = 250.0;        // 通信有效范围（米）
const double TOPOLOGY_UPDATE_INTERVAL = 5.0;  // 拓扑更新间隔（秒）
const double PACKET_INTERVAL = 0.5;     // 数据包发送间隔（秒）

NodeContainer nodes;
std::ofstream topologyFile("topology-changes.txt");
std::ofstream transmissionFile("node-transmissions.txt");
std::map<std::pair<uint32_t, uint32_t>, bool> activeLinks; // 当前活动链路
ApplicationContainer clientApps;

// // 三维距离计算函数
// double CalculateDistance(Vector a, Vector b) {
//     return std::sqrt(std::pow(a.x-b.x,2) + std::pow(a.y-b.y,2) + std::pow(a.z-b.z,2));
// }

uint32_t GetNodeIdByIp(Ipv4Address ip) {
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        if (nodes.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal() == ip) {
            return i;
        }
    }
    return UINT32_MAX;
}

uint32_t GetNodeIdFromContext(const std::string &context) {
    // context 形如 "/NodeList/2/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx"
    // 我们只要抓出中间那个 "2" 就行
    std::string prefix = "/NodeList/";
    size_t startPos = context.find(prefix);
    if (startPos == std::string::npos) {
        return 0;
    }
    startPos += prefix.size();  // 跳过 "/NodeList/"
    size_t endPos = context.find('/', startPos);
    // endPos - startPos 就是节点编号的字符串长度
    std::string nodeIdStr = context.substr(startPos, endPos - startPos);
    return std::stoul(nodeIdStr);  // 转成 uint32_t
}


// 更新拓扑结构（基于实际位置）
void UpdateTopology(NodeContainer& nodes) {
    activeLinks.clear();
    std::vector<Ptr<MobilityModel>> mob(nodes.GetN());
    
    // 获取所有节点位置
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        mob[i] = nodes.Get(i)->GetObject<MobilityModel>();
    }

    // 检测有效通信链路
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        for (uint32_t j = i+1; j < nodes.GetN(); ++j) {
            double distance = CalculateDistance(mob[i]->GetPosition(), mob[j]->GetPosition());
            if (distance <= COMM_RANGE) {
                activeLinks[{i,j}] = true;
                activeLinks[{j,i}] = true; // 双向链路
            }
        }
    }

    // 记录拓扑变化
    double timeNow = Simulator::Now().GetSeconds();
    topologyFile << "Time: " << timeNow << "s | Active Links: ";
    for (auto& link : activeLinks) {
        if(link.second) {
            topologyFile << link.first.first << "<->" << link.first.second << " ";
        }
    }
    topologyFile << "\n";
}

// 记录传输事件（包括ACK）
void LogTransmission(uint32_t nodeId, const std::string& type) {
    Ptr<Node> node = nodes.Get(nodeId);
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();
    
    transmissionFile << Simulator::Now().GetSeconds() << ","
                    << nodeId << ","
                    << type << ","
                    << pos.x << "," << pos.y << "," << pos.z << "\n";
}

// 数据包发送回调
void TxTrace(std::string context, Ptr<const Packet> packet) {
    uint32_t nodeId = GetNodeIdFromContext(context);
    LogTransmission(nodeId, "DATA");
}

// // ACK接收回调（修正参数顺序）
// void ReceiveAck(Ptr<const Packet> pkt, const Address& srcAddr, const Address& dstAddr) {
//     // InetSocketAddress srcInet = InetSocketAddress::ConvertFrom(srcAddr);
//     InetSocketAddress dstInet = InetSocketAddress::ConvertFrom(dstAddr);
    
//     // 记录ACK发送节点（服务器端）
//     for (uint32_t i = 0; i < nodes.GetN(); ++i) {
//         if (nodes.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal() == dstInet.GetIpv4()) {
//             LogTransmission(i, "ACK");
//             break;
//         }
//     }
// }

// // 周期发送数据包
// void ScheduleTransmissions() {
//     for (auto& link : activeLinks) {
//         if(link.second) {
//             uint32_t src = link.first.first;
//             uint32_t dst = link.first.second;

//             // 创建UDP客户端应用
//             UdpEchoClientHelper client(nodes.Get(dst)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(), 2000);
//             client.SetAttribute("MaxPackets", UintegerValue(1));
//             client.SetAttribute("Interval", TimeValue(Seconds(PACKET_INTERVAL)));
//             client.SetAttribute("PacketSize", UintegerValue(512));
            
//             ApplicationContainer app = client.Install(nodes.Get(src));
//             app.Start(Simulator::Now());
//             app.Stop(Simulator::Now() + Seconds(PACKET_INTERVAL * 0.9));
//             clientApps.Add(app);
//         }
//     }
//     // 递归调度
//     Simulator::Schedule(Seconds(PACKET_INTERVAL), &ScheduleTransmissions);
// }

void ClientReceiveAck(Ptr<const Packet> packet, const Address& address) {
    InetSocketAddress inetAddr = InetSocketAddress::ConvertFrom(address);
    uint32_t nodeId = GetNodeIdByIp(inetAddr.GetIpv4());
    if(nodeId != UINT32_MAX) {
        LogTransmission(nodeId, "ACK_RECEIVED");
    }
}

// 服务器接收数据包回调（更名为ServerReceive）
void ServerReceive(Ptr<const Packet> pkt, const Address& srcAddr, const Address& dstAddr) {
    InetSocketAddress srcInet = InetSocketAddress::ConvertFrom(srcAddr);
    uint32_t srcNodeId = GetNodeIdByIp(srcInet.GetIpv4());
    if(srcNodeId != UINT32_MAX) {
        LogTransmission(srcNodeId, "ACK");
    }
}

// 更新客户端应用创建逻辑
void CreateClientApplication(uint32_t src, uint32_t dst) {
    UdpEchoClientHelper client(nodes.Get(dst)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(), 2000);
    client.SetAttribute("MaxPackets", UintegerValue(1));
    client.SetAttribute("Interval", TimeValue(Seconds(PACKET_INTERVAL)));
    client.SetAttribute("PacketSize", UintegerValue(512));
    
    ApplicationContainer app = client.Install(nodes.Get(src));
    app.Start(Simulator::Now());
    app.Stop(Simulator::Now() + Seconds(PACKET_INTERVAL * 0.9));
    
    // 绑定客户端接收回调
    Ptr<UdpEchoClient> clientApp = app.Get(0)->GetObject<UdpEchoClient>();
    clientApp->TraceConnectWithoutContext("Received", MakeCallback(&ClientReceiveAck));
    
    clientApps.Add(app);
}

// 修改后的周期发送函数
void ScheduleTransmissions() {
    for (auto& link : activeLinks) {
        if(link.second) {
            CreateClientApplication(link.first.first, link.first.second);
            CreateClientApplication(link.first.second, link.first.first); // 双向通信
        }
    }
    Simulator::Schedule(Seconds(PACKET_INTERVAL), &ScheduleTransmissions);
}

int main(int argc , char *argv[]) {
    uint32_t numNodes = 20;
    double simulationTime = 60.0;
    SeedManager::SetSeed(12345);

    nodes.Create(numNodes);

    // 三维移动模型配置
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=" + std::to_string(SIM_AREA_SIZE) + "]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=" + std::to_string(SIM_AREA_SIZE) + "]"),
        "Z", StringValue("ns3::UniformRandomVariable[Min=50|Max=150]"));
    mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
        "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=10|Max=20]"),
        "Bounds", BoxValue(Box(0, SIM_AREA_SIZE, 0, SIM_AREA_SIZE, 50, 150)));
    mobility.Install(nodes);

    // 无线网络配置
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.Set("TxPowerStart", DoubleValue(23.0));
    phy.Set("TxPowerEnd", DoubleValue(23.0));
    phy.Set("RxSensitivity", DoubleValue(-85.0)); // 接收灵敏度
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac",
               "QosSupported", BooleanValue(true));
    
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ac);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // 协议栈配置
    InternetStackHelper stack;
    AodvHelper aodv;
    stack.SetRoutingHelper(aodv);
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // 初始化ACK服务器
    UdpEchoServerHelper ackServer(2000);
    ApplicationContainer servers = ackServer.Install(nodes);
    servers.Start(Seconds(0.0));
    servers.Stop(Seconds(simulationTime));

    // 绑定回调函数
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&TxTrace));
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<UdpEchoServer> server = servers.Get(i)->GetObject<UdpEchoServer>();
        server->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&ServerReceive)); // 更名为ServerReceive
    }

    // 调度拓扑更新和包发送
    Simulator::Schedule(Seconds(0.1), &UpdateTopology, nodes);
    Simulator::Schedule(Seconds(0.1), &ScheduleTransmissions);
    for (double t = TOPOLOGY_UPDATE_INTERVAL; t < simulationTime; t += TOPOLOGY_UPDATE_INTERVAL) {
        Simulator::Schedule(Seconds(t), &UpdateTopology, nodes);
    }

    // 流量监控
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    // 结果输出
    monitor->SerializeToXmlFile("uav-flowmon.xml", true, true);
    topologyFile.close();
    transmissionFile.close();
    Simulator::Destroy();
    return 0;
}