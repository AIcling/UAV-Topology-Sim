#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavAdhocNetwork");

const double SIM_AREA_SIZE = 500.0;
const double UAV_SPEED = 15.0;
const double TIME_SLOT = 0.1;

NodeContainer nodes; // 全局定义nodes
std::ofstream outFile("uav-packet-sent.txt", std::ios::out); 

uint32_t GetNodeIdFromContext(std::string context) {
    std::size_t n1 = context.find("/NodeList/") + 10;
    std::size_t n2 = context.find("/", n1);
    return std::stoi(context.substr(n1, n2 - n1));
}

void ScheduleTxSlots(NodeContainer& nodes) {
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<Node> node = nodes.Get(i);
        Simulator::Schedule(Seconds(i * TIME_SLOT), [node](){
            Ptr<Application> app = node->GetApplication(0);
            if(app) {
                Ptr<OnOffApplication> onoff = DynamicCast<OnOffApplication>(app);
                if(onoff) {
                    onoff->SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.05]"));
                    onoff->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.95]"));
                }
            }
        });
    }
}

void TxTrace(std::string context, Ptr<const Packet> packet) {
    double timeNow = Simulator::Now().GetSeconds();
    uint32_t nodeId = GetNodeIdFromContext(context);
    Vector pos = nodes.Get(nodeId)->GetObject<MobilityModel>()->GetPosition();
    outFile << "Time: " << timeNow << "s, Node ID: " << nodeId
            << ", Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
}

int main(int argc, char *argv[]) {
    uint32_t numNodes = 20;
    double simulationTime = 60.0;

    nodes.Create(numNodes);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=" + std::to_string(SIM_AREA_SIZE) + "]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=" + std::to_string(SIM_AREA_SIZE) + "]"),
        "Z", StringValue("ns3::UniformRandomVariable[Min=50|Max=150]"));

    mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
        "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min="+std::to_string(UAV_SPEED-5)+"|Max="+std::to_string(UAV_SPEED+5)+"]"),
        "Bounds", BoxValue(Box(0, SIM_AREA_SIZE, 0, SIM_AREA_SIZE, 50, 150)));
    mobility.Install(nodes);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(5.0e9));

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(23.0));
    phy.Set("TxPowerEnd", DoubleValue(23.0));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac",
        "QosSupported", BooleanValue(true),
        "BE_MaxAmpduSize", UintegerValue(65535),
        "BE_MaxAmsduSize", UintegerValue(3839));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ac);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    InternetStackHelper stack;
    AodvHelper aodv;
    stack.SetRoutingHelper(aodv);
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    for (uint32_t i = 0; i < numNodes; ++i) {
        OnOffHelper client("ns3::UdpSocketFactory", Address(InetSocketAddress(interfaces.GetAddress((i+1)%numNodes), 50000)));
        client.SetAttribute("PacketSize", UintegerValue(512));
        client.SetAttribute("DataRate", DataRateValue(DataRate("2Mbps")));

        ApplicationContainer clientApps = client.Install(nodes.Get(i));
        clientApps.Start(Seconds(1.0 + i*0.1));
        clientApps.Stop(Seconds(simulationTime - 1));
    }

    ScheduleTxSlots(nodes);

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&TxTrace));

    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    monitor->SerializeToXmlFile("uav-flowmon.xml", true, true);
    outFile.close(); // 关闭文件
    Simulator::Destroy();
    return 0;
}
