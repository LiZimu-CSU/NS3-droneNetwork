#include "ns3/tap-bridge-module.h"
#include "ns3/abort.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/dsr-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include <iostream>
#include <istream>
#include "ns3/ssid.h"
#include <string.h>
#include "ns3/propagation-loss-model.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <arpa/inet.h>
#include "ns3/ppp-header.h"
#include <netinet/in.h>
#include <map>

using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE("PingEmulationExample");



//下面是传输数据用的结构体，有两种数据格式，都需要监听并识别
struct SOut2Simulator
{
    int copterID;
    int vehicleType;
    double runnedTime;
    float VelE[3];
    float PosE[3];
    float AngEuler[3];
    float AngQuatern[4];
    float MotorRPMS[8];
    float AccB[3];
    float RateB[3];
    double PosGPS[3];
    SOut2Simulator()
    {
        reset();
    }
    void reset()
    {
        copterID = -1;
        vehicleType = -1;
        runnedTime = 0;
        for (int i = 0; i < 3; i++)
        {
            VelE[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            PosE[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            AngEuler[i] = 0;
        }
        for (int i = 0; i < 4; i++)
        {
            AngQuatern[i] = 0;
        }
        for (int i = 0; i < 8; i++)
        {
            MotorRPMS[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            AccB[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            RateB[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            PosGPS[i] = 0;
        }
    }
};

#define PAYLOAD_LEN_SHORT 192

typedef struct _netDataShort
{
    int tg;
    int len;
    char payload[PAYLOAD_LEN_SHORT];
    _netDataShort()
    {
        memset(payload, 0, sizeof(payload));
    }
} netDataShort;

struct SOut2SimulatorSimpleTime
{
    int checkSum;
    int copterID;    // Vehicle ID
    int vehicleType; // Vehicle type
    float PWMs[8];
    float PosE[3]; // NED vehicle position in earth frame (m)
    float VelE[3];
    float AngEuler[3]; // Vehicle Euler angle roll pitch yaw (rad) in x y z
    double runnedTime; // Current Time stamp (ms)
    SOut2SimulatorSimpleTime()
    {
        reset();
    }
    void reset()
    {
        checkSum = -1;
        copterID = -1;
        vehicleType = -1;
        runnedTime = -1;
        for (int i = 0; i < 8; i++)
        {
            PWMs[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            PosE[i] = 0;
            AngEuler[i] = 0;
            VelE[i] = 0;
        }
    }
};


//两个子网卡,enp3s0:0绑定192.168.31.180,enp3s0:1绑定192.168.31.
// 180,181均为虚拟服务端,虚拟节点
string deviceName("ens33");
string localAddress("192.168.31.142");
string localGateway("192.168.31.1");
std::string mode = "ConfigureLocal";
std::string tapName = "thetap";
Ipv4Address localIp(localAddress.c_str());
Ipv4Mask localMask("255.255.255.0");

//服务端接收到数据包receivenum++，客户端发送一次数据包sendnum++
uint32_t sendnum = 0;
uint32_t receivenum = 0;
int uav = 1;
NodeContainer receiver;
NodeContainer uavs;
Ptr<Socket> send_sock;
struct destIPAndPort{
    uint8_t ip[4];
    uint32_t port;
    uint16_t copterID;
    uint8_t reserved[6];
} dest_ip_port;
map<Ipv4Address, uint16_t> IpIdMap;
map<uint16_t, Ipv4Address> IdIpMap;
uint16_t copterNumber = 0;

void receivePacket(Ptr<Socket> sock);
void receivePacketFromNx(Ptr<Socket> sock);
void receiveMulicastPacketFromNx(Ptr<Socket> sock);
void addNewNodeToUavs();
void ReceiveWin(Ptr<Node> nodelist[], int uavNum, string context, const Ptr<const Packet> packet);

//parameters that needed in initializing a new node with the same settign in the uav network
WifiHelper wifi;
YansWifiPhyHelper phy;
WifiMacHelper mac;
InternetStackHelper stackUavs;
MobilityHelper mobility;
Ipv4AddressHelper addressUavs;
TypeId tid;


int main(int argc, char *argv[])
{
    //作用仅仅只是开启可视化功能不用管
    CommandLine cmd(__FILE__);
    cmd.AddValue("uav", "the number of uav nodes you want to create", uav);
    cmd.Parse(argc, argv);
    
    // GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    // GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true)); 
    receiver.Create(1);

    FdNetDeviceHelper *helper = nullptr;    
    EmuFdNetDeviceHelper *raw = new EmuFdNetDeviceHelper;
    raw->SetDeviceName(deviceName);
    raw->SetAttribute ("EncapsulationMode", StringValue ("Dix"));
    helper = raw;

    NetDeviceContainer fd = helper->Install(receiver.Get(0));
    Ptr<NetDevice> fddevice = fd.Get(0);
    fddevice->SetAttribute("Address", Mac48AddressValue(Mac48Address::Allocate()));
    
    InternetStackHelper stack;
    stack.Install(receiver);

    Ptr<Ipv4> ipv4 = receiver.Get(0)->GetObject<Ipv4>();
    uint32_t interface = ipv4->AddInterface(fddevice);
    Ipv4InterfaceAddress address = Ipv4InterfaceAddress(localIp, localMask);
    ipv4->AddAddress(interface, address);
    ipv4->SetMetric(interface, 1);
    ipv4->SetUp(interface);

    UdpEchoServerHelper echosever_Win(20009);
    ApplicationContainer severapp_Win;
    severapp_Win = echosever_Win.Install(receiver.Get(0));
    severapp_Win.Start(Seconds(1.0));
    severapp_Win.Stop(Seconds(500.0));  

    //Here is  an uav network
    uavs.Create(uav);


    // WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");
    // YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange", DoubleValue(500));
    phy.SetChannel(channel.Create());
    // WifiMacHelper mac;
    Ssid ssid;
    mac.SetType("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer devices = wifi.Install(phy, mac, uavs);

    // MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(50),
                                  "DeltaY", DoubleValue(50),
                                  "GridWidth", UintegerValue(uav),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(uavs);

    // InternetStackHelper stackUavs;
    Ipv4ListRoutingHelper listRouting;
    OlsrHelper olsr;
    listRouting.Add(olsr,10);
    stackUavs.SetRoutingHelper(listRouting);
    stackUavs.Install(uavs);

    // Ipv4AddressHelper addressUavs;
    addressUavs.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = addressUavs.Assign(devices);

    //Let receiver's echoSeverApplication receiced a packet to send into uav network
    Ptr<Node> nodelist[uav];
    for(int i=0; i<uav; i++){
        nodelist[i] = uavs.Get(i);
    }
    stringstream os1;
    os1 << "/NodeList/" << receiver.Get(0)->GetId() << "/ApplicationList/0/$ns3::UdpEchoServer/Rx";
    Config::Connect(os1.str(), MakeBoundCallback(&ReceiveWin, nodelist, uav));


    tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    send_sock = Socket::CreateSocket(receiver.Get(0), tid);
    // send_sock->SetAllowBroadcast(true);
    // InetSocketAddress addr = InetSocketAddress(Ipv4Address("192.168.31.9"), 10100);
    // send_sock->Connect(addr);

    //make uavs to tansmit and recieve packet from each other
    for(int i = 0 ; i< uav ; i++){
        Ptr<Socket> recv_socket = Socket::CreateSocket(uavs.Get(i), tid);
        InetSocketAddress addr = InetSocketAddress(interfaces.GetAddress(i) , 20000);
        recv_socket->Bind(addr);
        recv_socket->SetRecvCallback(MakeCallback(&receivePacket));
    }
    Ptr<Socket> recv_socket = Socket::CreateSocket(receiver.Get(0), tid);
    InetSocketAddress addr2 = InetSocketAddress("224.0.0.10" , 9000);
    recv_socket->Bind(addr2);
    recv_socket->SetRecvCallback(MakeCallback(&receivePacketFromNx));

    Ptr<Socket> recv_socket3 = Socket::CreateSocket(receiver.Get(0), tid);
    InetSocketAddress addr23 = InetSocketAddress("224.0.0.10" , 9100);
    recv_socket3->Bind(addr23);
    recv_socket3->SetRecvCallback(MakeCallback(&receiveMulicastPacketFromNx));

    
    // start simulation
    Simulator::Stop(Seconds(600.0));
    Simulator::Run();
    Simulator::Destroy();
 
    delete helper;
}

void receivePacket(Ptr<Socket> sock){
    Address from;
    Ptr<Packet> packet = sock->RecvFrom(from);
    packet->CopyData((uint8_t *)&dest_ip_port, 16);
    packet->RemoveAtStart(16);
    Ipv4Address temp;
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    InetSocketAddress dest_addr = InetSocketAddress(temp.Deserialize(dest_ip_port.ip), uint16_t(dest_ip_port.port)) ;
    Ptr<Socket> sender = Socket::CreateSocket(receiver.Get(0), tid);
    //cout << "dest   : "<< temp.Deserialize(dest_ip_port.ip) << "  : "<< uint16_t(dest_ip_port.port)<< endl;
    sender->Bind(from);
    sender->Connect(dest_addr);
    sender->Send(packet);
    sender->Close();
    // cout << sender->Send(packet) << "this is from receivePacket()" <<  endl;
    
    // cout << "received" << endl;
    
}

void receivePacketFromNx(Ptr<Socket> sock){
    Address from;
    Ptr<Packet> packet = sock->RecvFrom(from);
    //we need to get the source IP and port 
    InetSocketAddress sourece_address = InetSocketAddress::ConvertFrom(from);
    packet->CopyData((uint8_t *)&dest_ip_port, 8);
    Ipv4Address temp = temp.Deserialize(dest_ip_port.ip);

    //This if segment is never executed!
    if(!IpIdMap[sourece_address.GetIpv4()] || !IpIdMap[temp]){
        cout << "map is not initialized at present!" << endl;
        return ;
    }
    // cout << "recieve packet from nx"<< endl;
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sender = Socket::CreateSocket(uavs.Get(IpIdMap[sourece_address.GetIpv4()]-1), tid);
    Ptr<Ipv4> ippp = uavs.Get(IpIdMap[temp]-1)-> GetObject<Ipv4> ();
    temp = ippp->GetAddress(1,0).GetLocal();
    InetSocketAddress dest_addr = InetSocketAddress(temp, 20000);
    sender->Bind();
    sender->Connect(dest_addr);
    sender->Send(packet) ;
    sender->Close();
}


void receiveMulicastPacketFromNx(Ptr<Socket> sock){
    Address from;
    Ptr<Packet> packet = sock->RecvFrom(from);
    InetSocketAddress sourece_address = InetSocketAddress::ConvertFrom(from);
    packet->CopyData((uint8_t *)&dest_ip_port, 16);
    packet->RemoveAtStart(16);
//construct the IPIDMap and IDIPMap here

    if(!IpIdMap[sourece_address.GetIpv4()]){
        copterNumber++;
        // consider one more copter take part in the drome network
        if(copterNumber > uav){
            //add a node to uav nodecontainer
            addNewNodeToUavs();
        }
        IpIdMap[sourece_address.GetIpv4()] = dest_ip_port.copterID;
        IdIpMap[dest_ip_port.copterID] = sourece_address.GetIpv4();
    }



    Ipv4Address temp;
    temp = temp.Deserialize(dest_ip_port.ip);
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sender = Socket::CreateSocket(receiver.Get(0), tid);
    InetSocketAddress dest_addr = InetSocketAddress("255.255.255.255", dest_ip_port.port);
    sender->SetAllowBroadcast(true);
    sender->Bind(sourece_address);
    sender->Connect(dest_addr);
    sender->Send(packet);
    sender->Close();
}

void addNewNodeToUavs(){
    NodeContainer newNode(1);
    NetDeviceContainer newNodeDevice = wifi.Install(phy, mac, newNode);
    mobility.Install(newNode);
    stackUavs.Install(newNode);
    Ipv4InterfaceContainer newNodeInterface = addressUavs.Assign(newNodeDevice);
    cout << "This is from addNode"<< endl;//
    mobility.Install(newNode);
    Ptr<Socket> recv_socket = Socket::CreateSocket(newNode.Get(0), tid);
    InetSocketAddress addr = InetSocketAddress(newNodeInterface.GetAddress(0) , 20000);
    recv_socket->Bind(addr);
    recv_socket->SetRecvCallback(MakeCallback(&receivePacket));
    uavs.Add(newNode);
    cout << uavs.GetN() << endl;
}

void ReceiveWin(Ptr<Node> nodelist[], int nodes_num, string context, const Ptr<const Packet> packet)
{
    //这里是接收所有飞机位置的监听函数
    //下面是解析Windows发送的UDP消息的代码
    int num = packet->GetSize();
    // uint8* RecvData = packet.GetData;

    // uint8_t *buffer = new uint8_t[packet->GetSize()];
    // packet->CopyData(buffer, packet->GetSize());
    //解析SOut2Simulator的数据包
    if (num == sizeof(netDataShort))
    {
        // cout << "success to receive WIN" << endl;
        receivenum++;
        // receivetime = Simulator::Now();
        netDataShort data;
        packet->CopyData((uint8_t *)&data, packet->GetSize());
        // memcpy(&data, RecvData, ArrayReader->Num());
        SOut2Simulator data3d;
        if (data.len == sizeof(data3d))
        {
            memcpy(&data3d, data.payload, data.len);
            int CopterID = data3d.copterID;
            if (CopterID > 0 && CopterID <= nodes_num)
            {
                //首先判断收到的CopterID是否已创建节点，如果没有则跳过
                Ptr<ConstantVelocityMobilityModel> mob = nodelist[CopterID - 1]->GetObject<ConstantVelocityMobilityModel>(); //获取CopterID对应的Nodes，例如1号飞机对应Node.get(0)
                mob->SetVelocity(Vector(data3d.VelE[0], data3d.VelE[1], 0));
                mob->SetPosition(Vector(data3d.PosE[0], data3d.PosE[1], 0));
                // Mobilitychange(mob, data3d.VelE[0], data3d.VelE[1], data3d.VelE[2], data3d.PosE[0], data3d.PosE[1], data3d.PosE[2]); //配置CopterID号飞机的位置，注意Z轴调整为向上为正
            }
        }
        return;
    }
    if (num == sizeof(SOut2SimulatorSimpleTime))
    {
        struct SOut2SimulatorSimpleTime data3d;
        // memcpy(&data3d, RecvData, ArrayReader->Num());
        packet->CopyData((uint8_t *)&data3d, packet->GetSize());
        if (data3d.checkSum == 1234567890)
        {
            int CopterID = data3d.copterID;
            if (CopterID > 0 && CopterID <= nodes_num)
            {                                                                                                                //首先判断收到的CopterID是否已创建节点，如果没有则跳过
                Ptr<ConstantVelocityMobilityModel> mob = nodelist[CopterID - 1]->GetObject<ConstantVelocityMobilityModel>(); //获取CopterID对应的Nodes，例如1号飞机对应Node.get(0)
                mob->SetVelocity(Vector(data3d.VelE[0], data3d.VelE[1], 0));
                mob->SetPosition(Vector(data3d.PosE[0], data3d.PosE[1], 0));
                // Mobilitychange(mob, data3d.VelE[0], data3d.VelE[1], -data3d.VelE[2], data3d.PosE[0], data3d.PosE[1], -data3d.PosE[2]); //配置CopterID号飞机的位置，注意Z轴调整为向上为正
                cout << "Receive SOut2SimulatorSimpleTime data" << data3d.VelE[0] << data3d.VelE[1] << -data3d.VelE[2] << data3d.PosE[0] << data3d.PosE[1] << -data3d.PosE[2] << endl;
            }
            return;
        }
    }
}
