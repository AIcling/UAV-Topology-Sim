#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include <fstream>
#include <iostream>
#include <string>
#include <set>
#include <map>
#include <algorithm>
#include <iomanip>


using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("UavSimulation");

// 全局文件流用于记录事件
static ofstream g_transFile;
static ofstream g_topoFile;
// 链路集合数组（10个区间）
static std::vector< std::set< std::pair<uint32_t,uint32_t> > > g_intervalLinks(10);
// IP地址到节点ID的映射表
static std::map<uint32_t, uint32_t> g_ipToNodeId;



// IPv4发送事件回调
static void Ipv4Tracer(std::string context, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    // 提取 IP 头部
    Ptr<Packet> pktCopy = packet->Copy();
    Ipv4Header ipHeader;
    pktCopy->RemoveHeader(ipHeader);

    if (ipHeader.GetProtocol() != 6) return; // 只处理 TCP 包

    TcpHeader tcpHeader;
    pktCopy->RemoveHeader(tcpHeader);

    bool isAck = tcpHeader.GetFlags() & TcpHeader::ACK;
    uint32_t payloadSize = pktCopy->GetSize();

    if (payloadSize == 0 && !isAck) return; // SYN/FIN控制包，不记录

    // 提取当前节点 ID（从 context 中解析）
    uint32_t nodeId = 0;
    size_t pos1 = context.find("/NodeList/") + 10;
    size_t pos2 = context.find("/", pos1);
    if (pos1 != std::string::npos && pos2 != std::string::npos) {
        nodeId = atoi(context.substr(pos1, pos2 - pos1).c_str());
    }

    // 判断是发送还是接收：Tx 或 Rx 会分别传不同 context
    std::string eventType;
    if (context.find("/Tx") != std::string::npos) {
        eventType = (payloadSize > 0) ? "Tx Data" : "Tx Ack";
    } else {
        eventType = (payloadSize > 0) ? "Rx Data" : "Rx Ack";
    }

    // 写入 transmission 文件
    g_transFile << std::fixed << std::setprecision(3)
                << Simulator::Now().GetSeconds() << "s "
                << "Node" << nodeId << " " << eventType << std::endl;

    // 推断通信对端：根据 IP 地址映射节点 ID
    uint32_t peerNodeId = 0;
    Ipv4Address peerIp = (context.find("/Tx") != std::string::npos) ?
        ipHeader.GetDestination() : ipHeader.GetSource();
    auto it = g_ipToNodeId.find(peerIp.Get());
    if (it != g_ipToNodeId.end()) {
        peerNodeId = it->second;
    }

    if (peerNodeId != nodeId) {
        uint32_t a = std::min(nodeId, peerNodeId);
        uint32_t b = std::max(nodeId, peerNodeId);
        uint32_t idx = std::min((uint32_t)std::floor(Simulator::Now().GetSeconds() / 10.0), (uint32_t)9);
        g_intervalLinks[idx].insert(std::make_pair(a, b));
    }
}


// 每10秒调用，输出拓扑活动链路
static void TopologyOutput(uint32_t index)
{
    double start = index * 10.0;
    double end = start + 10.0;
    // 格式化输出时间段
    g_topoFile << std::fixed << std::setprecision(0) 
               << start << "-" << end << "s: ";
    if (g_intervalLinks[index].empty()) {
        g_topoFile << "none";
    } else {
        bool first = true;
        for (auto pr : g_intervalLinks[index]) {
            if (!first) {
                g_topoFile << ", ";
            }
            g_topoFile << "Node" << pr.first << "-Node" << pr.second;
            first = false;
        }
    }
    g_topoFile << std::endl;
    // 清空当前区间集合，为下一个区间重新统计
    g_intervalLinks[index].clear();
}

int main(int argc, char *argv[])
{
    // 创建20个节点
    NodeContainer nodes;
    nodes.Create(20);

    LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
    LogComponentEnable("PacketSink", LOG_LEVEL_INFO);

    // 配置移动模型: Gauss-Markov Mobility Model 三维随机移动
    MobilityHelper mobility;
    // 初始位置分布在立方体范围内随机均匀
    mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"),
        "Z", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=200.0]")
    );
    // 设置GaussMarkov模型参数
    mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
        "Bounds", BoxValue(Box(0, 500, 0, 500, 0, 100)),
        "TimeStep", TimeValue(Seconds(1.0)),
        "Alpha", DoubleValue(0.7),
        "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=20.0]"),
        "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283185]"),
        "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=0.0]"),
        "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=2.0]"),
        "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.5|Bound=1.0]"),
        "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.1|Bound=0.2]")
    );
    mobility.Install(nodes);

    // 配置Wi-Fi 802.11ac Adhoc通信
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ac);
    // 速率控制:使用固定速率避免速率自动调整的不确定性
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("VhtMcs0"), 
                                 "ControlMode", StringValue("VhtMcs0"));
    // 物理层及信道设置
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> wifiChannel = channel.Create ();

    // 创建 YansWifiPhyHelper
    YansWifiPhyHelper phy;
    // 将上面创建的 WifiChannel 绑定到 phy
    phy.SetChannel (wifiChannel);

    // 设置发射功率 28 dBm，接收门限 -90 dBm
    phy.Set ("TxPowerStart", DoubleValue (28.0));
    phy.Set ("TxPowerEnd", DoubleValue (28.0));
    phy.Set("RxSensitivity", DoubleValue(-90.0));
    // MAC层设置为Adhoc模式
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    // 安装Wi-Fi设备到节点
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // 安装TCP/IP协议栈
    InternetStackHelper stack;
    stack.Install(nodes);
    // 分配IP地址
    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);
    // 填充IP映射表
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ipv4Address ip = interfaces.GetAddress(i);
        uint32_t nodeId = nodes.Get(i)->GetId();
        g_ipToNodeId[ip.Get()] = nodeId;
    }

    // 配置应用层：每个节点安装一个TCP PacketSink作为接收者
    uint16_t sinkPort = 9999;
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), sinkPort));
    ApplicationContainer sinkApps = sinkHelper.Install(nodes);
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(100.0));

    // 配置发送端应用：OnOffApplication随机启动TCP会话
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    rand->SetStream(1);  // 固定随机流以重现实验（可选）
    OnOffHelper onoff("ns3::TcpSocketFactory", Address());
    onoff.SetAttribute("PacketSize", UintegerValue(512));
    onoff.SetAttribute("DataRate", StringValue("10Mbps"));  // 高速率确保能发完
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));  // 非常短的窗口
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
    LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
    LogComponentEnable("PacketSink", LOG_LEVEL_INFO);   

    for (uint32_t t = 0; t < 100; ++t) {
        // 每秒都有一定概率触发一次通信
        if (rand->GetValue() < 0.5) {  // 20% 概率
            // 随机选择发送节点和接收节点（确保不同）
            uint32_t sender = rand->GetInteger(0, nodes.GetN() - 1);
            uint32_t receiver = rand->GetInteger(0, nodes.GetN() - 1);
            while (receiver == sender) {
                receiver = rand->GetInteger(0, nodes.GetN() - 1);
            }
            // 设置应用的远端地址为接收节点的IP和端口
            Address remoteAddress(InetSocketAddress(interfaces.GetAddress(receiver), sinkPort));
            onoff.SetAttribute("Remote", AddressValue(remoteAddress));
            // 在发送方节点安装OnOff应用
            ApplicationContainer app = onoff.Install(nodes.Get(sender));
            // 设置应用启动时间为t秒
            app.Start(Seconds((double)t));
            // 停止时间为t+0.02秒，确保只发送一个包后停止
            app.Stop(Seconds(t + 0.02));
        }
    }

    // 打开输出文件
    g_transFile.open("node-transmissions.txt");
    g_topoFile.open("topology-changes.txt");
    // 连接IP层Tx和Rx跟踪器
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Tx", MakeCallback(&Ipv4Tracer));
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Rx", MakeCallback(&Ipv4Tracer));

    // 安排每10秒输出拓扑活动链路
    for (uint32_t idx = 0; idx < 10; ++idx) {
        Simulator::Schedule(Seconds((idx + 1) * 10.0), &TopologyOutput, idx);
    }

    // 运行仿真
    Simulator::Stop(Seconds(100.0));
    Simulator::Run();
    Simulator::Destroy();

    // 关闭文件
    g_transFile.close();
    g_topoFile.close();
    return 0;
}
