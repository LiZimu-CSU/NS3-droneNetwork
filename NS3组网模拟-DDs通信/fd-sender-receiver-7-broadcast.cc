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
//uav means the number of drones in the network at the program initialized.
int uav = 1;
//receiver is a node that receive and send packets to physical network, which maybe delete as optimization.
NodeContainer receiver;
//uavs is the nodes in the drone network, and the number of drones in the uavs may change as program running.
NodeContainer uavs;
//destIPAndPort is a struct to parse data that we load in the packet using DDS.
struct destIPAndPort{
    uint8_t ip[4];
    uint32_t port;
    uint16_t copterID;
    uint8_t reserved[6];
} dest_ip_port;
//IpIdMap is the map that map the NX's IP address to CopterID of node in drone network.
map<Ipv4Address, uint16_t> IpIdMap;
//IdIpMap is the map that map CopterID of node in drone network to the NX's IP address .
map<uint16_t, Ipv4Address> IdIpMap;
//copterNumber is a counter to counter the number of drones in the drone network, if it's bigger than uav, which means we should add a node into drone network.
uint16_t copterNumber = 0;
//parameters that needed in initializing a new node with the same settign in the drone network
//all of the parameters bellow are initialized at program start and used to initialize the drone network at first.
// They are also used to add a new node to the drone network to ensure the node can contact with the node initialized before.
WifiHelper wifi;
YansWifiPhyHelper phy;
WifiMacHelper mac;
InternetStackHelper stackUavs;
MobilityHelper mobility;
Ipv4AddressHelper addressUavs;
TypeId tid;


void receivePacket(Ptr<Socket> sock);
void receivePacketFromNx(Ptr<Socket> sock);
void receiveMulicastPacketFromNx(Ptr<Socket> sock);
void addNewNodeToUavs();
void receiveMultiPacketFromNodes(Ptr<Socket> sock);
void ReceiveWin(Ptr<Node> nodelist[], int uavNum, string context, const Ptr<const Packet> packet);


int main(int argc, char *argv[])
{
    //作用仅仅只是开启可视化功能不用管
    //read the parameter uav from commandline and parse the parameters.
    CommandLine cmd(__FILE__);
    cmd.AddValue("uav", "the number of uav nodes you want to create", uav);
    cmd.Parse(argc, argv);
    
    // GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    //make sure the packet chechsum isn't be 0x0000, and ensure packets sent from NS3 to NX be accept by DDS running on NX.
    GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true)); 

    //Create the receiver, and bind it to the Ethernet card with FdNetdevice.
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


    //Here is  an uav network
    uavs.Create(uav);


    // WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");
    // YansWifiPhyHelper phy;

    //set channel propagation loss model, and if you want to change the  communication distance between uavs in drone netwok
    //You can change this line "channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange", DoubleValue(500));" with DoubleValue(distance), in which distance is the communication distance you need.
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange", DoubleValue(50));
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


    //We have finished the network construction and we will using this network receive and send packet.
    //Let receiver's echoSeverApplication receiced a packet to send into uav network
    Ptr<Node> nodelist[uav];
    for(int i=0; i<uav; i++){
        nodelist[i] = uavs.Get(i);
    }
    //receive packets from Rflysim
    //The function ReceiveWin will set the mobility of each node in uavs 
    //However ,at present Receive will not change the new node add in the drone network. The ReceiveWin should be changed latter.
    UdpEchoServerHelper echosever_Win(20009);
    ApplicationContainer severapp_Win;
    severapp_Win = echosever_Win.Install(receiver.Get(0));
    severapp_Win.Start(Seconds(1.0));
    severapp_Win.Stop(Seconds(500.0));  
    stringstream os1;
    os1 << "/NodeList/" << receiver.Get(0)->GetId() << "/ApplicationList/0/$ns3::UdpEchoServer/Rx";
    Config::Connect(os1.str(), MakeBoundCallback(&ReceiveWin, nodelist, uav));


    tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    //make uavs to tansmit and recieve packet from each other
    //all of these sockets will letsen packet sent from other node in the drone network and the callback function will send the packet to the physical network which means send the packet to NX
    for(int i = 0 ; i< uav ; i++){
        Ptr<Socket> recv_socket = Socket::CreateSocket(uavs.Get(i), tid);
        InetSocketAddress addr = InetSocketAddress(interfaces.GetAddress(i) , 20000);
        recv_socket->Bind(addr);
        recv_socket->SetRecvCallback(MakeCallback(&receivePacket));

        Ptr<Socket> recvMulti_socket = Socket::CreateSocket(uavs.Get(i), tid);
        InetSocketAddress addr33 = InetSocketAddress("255.255.255.255" , 20100);
        recvMulti_socket->Bind(addr33);
        recvMulti_socket->SetRecvCallback(MakeCallback(&receiveMultiPacketFromNodes));
    }

    //this is a test that check multicast in drone network

    

    //receive unicast packets from NX
    Ptr<Socket> recv_socket = Socket::CreateSocket(receiver.Get(0), tid);
    InetSocketAddress addr2 = InetSocketAddress("224.0.0.10" , 9000);
    recv_socket->Bind(addr2);
    recv_socket->SetRecvCallback(MakeCallback(&receivePacketFromNx));

    //receive multicast packets sent from NX
    Ptr<Socket> receiveMulicastSocket = Socket::CreateSocket(receiver.Get(0), tid);
    InetSocketAddress addr23 = InetSocketAddress("224.0.0.10" , 9100);
    receiveMulicastSocket->Bind(addr23);
    receiveMulicastSocket->SetRecvCallback(MakeCallback(&receiveMulicastPacketFromNx));

    
    // start simulation
    Simulator::Stop(Seconds(600.0));
    Simulator::Run();
    Simulator::Destroy();
 
    delete helper;
}


//functio "void receivePacket(Ptr<Socket> sock)" will receive packets from other nodes in the drone network, it will create a socket of receiver and use which to send packets to physical network.
//it is a callback function of nodes' sockets in uavs(NodeContainer)
void receivePacket(Ptr<Socket> sock)
{
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


//function "void receivePacketFromNx(Ptr<Socket> sock)" will received unicast packets from NX and it will create sockets of uavs in drone network to send the packet to drone network.
//it is a callback function of receiver's socket.
void receivePacketFromNx(Ptr<Socket> sock)
{
    Address from;
    Ptr<Packet> packet = sock->RecvFrom(from);
    //we need to get the source IP and port 
    InetSocketAddress sourece_address = InetSocketAddress::ConvertFrom(from);
    packet->CopyData((uint8_t *)&dest_ip_port, 8);
    Ipv4Address temp = temp.Deserialize(dest_ip_port.ip);

    //This if segment will be executed several times!
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
    cout << sender->Send(packet) << endl;
    sender->Close();
}

void receiveMultiPacketFromNodes(Ptr<Socket> sock){
    cout << "successed" << endl;
    Ptr<Packet> packet = sock->Recv();
    Ptr<Node> destNode = sock->GetNode();
    Ipv4Address destAddress = IdIpMap[destNode->GetId()] ;
    Ptr<Socket> sender = Socket::CreateSocket(receiver.Get(0), tid);
    InetSocketAddress dest_addr = InetSocketAddress(destAddress, 7412);
    // sender->SetAllowBroadcast(true);
    // addressUtils
    // sender->Ipv6JoinGroup(v6Address);
    // sender->Bind(sourece_address);
    sender->Connect(dest_addr);
    // cout << "This is from receiveMulticastPacketFromNx :  "<<sender->Send(packet) << endl;
    sender->Send(packet);
    sender->Close();
}

//function "void receiveMulicastPacketFromNx(Ptr<Socket> sock)" will receive multicast packets from NX, these packets are all RTPS packets that help particepants contact with each other.
//it is a callback function of receiver's socket.
void receiveMulicastPacketFromNx(Ptr<Socket> sock)
{
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
            //add a node to uavs nodecontainer
            addNewNodeToUavs();
        }
        IpIdMap[sourece_address.GetIpv4()] = dest_ip_port.copterID;
        IdIpMap[dest_ip_port.copterID] = sourece_address.GetIpv4();
    }

// static Ipv6Address MakeIpv4MappedAddress (Ipv4Address addr);
    // Ipv6Address v6Address = Ipv6Address::MakeIpv4MappedAddress (Ipv4Address("224.0.0.10"));

    Ipv4Address temp;
    temp = temp.Deserialize(dest_ip_port.ip);
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sender = Socket::CreateSocket(uavs.Get(0), tid);
    InetSocketAddress dest_addr = InetSocketAddress("255.255.255.255", 20100);
    sender->SetAllowBroadcast(true);
    // addressUtils
    // sender->Ipv6JoinGroup(v6Address);
    sender->Bind(sourece_address);
    sender->Connect(dest_addr);
    // cout << "This is from receiveMulticastPacketFromNx :  "<<sender->Send(packet) << endl;
    sender->Send(packet);
    sender->Close();
}


//fuction "void addNewNodeToUavs()" will add a new node into drone network
//It will be only called in function receiveMulticastPacketFromNX
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
    Ptr<Socket> recvMulti_socket = Socket::CreateSocket(newNode.Get(0), tid);
    InetSocketAddress addr33 = InetSocketAddress("255.255.255.255" , 20100);
    recvMulti_socket->Bind(addr33);
    recvMulti_socket->SetRecvCallback(MakeCallback(&receiveMultiPacketFromNodes));
    uavs.Add(newNode);
    cout << uavs.GetN() << endl;
}



//function "void ReceiveWin(Ptr<Node> nodelist[], int nodes_num, string context, const Ptr<const Packet> packet)" will receive packets from Rflysim and change the mobility model of uavs in drone network;
//however this function will not change the new nodes' mobility model, and this function will be rewrite latter.
//it is a callback function of receiver's echo server application.
void ReceiveWin(Ptr<Node> nodelist[], int uavNum, string context, const Ptr<const Packet> packet)
{
    //这里是接收所有飞机位置的监听函数
    //下面是解析Windows发送的UDP消息的代码
    int num = packet->GetSize();
    int nodes_num = uavs.GetN();
    // NodeContainer
    //解析SOut2Simulator的数据包
    if (num == sizeof(netDataShort))
    {
        receivenum++;
        netDataShort data;
        packet->CopyData((uint8_t *)&data, packet->GetSize());
        SOut2Simulator data3d;
        if (data.len == sizeof(data3d))
        {
            memcpy(&data3d, data.payload, data.len);
            int CopterID = data3d.copterID;
            if (CopterID > 0 && CopterID <= nodes_num)
            {
                //首先判断收到的CopterID是否已创建节点，如果没有则跳过
                Ptr<ConstantVelocityMobilityModel> mob = uavs.Get(CopterID - 1)->GetObject<ConstantVelocityMobilityModel>(); //获取CopterID对应的Nodes，例如1号飞机对应Node.get(0)
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
                Ptr<ConstantVelocityMobilityModel> mob = uavs.Get(CopterID - 1)->GetObject<ConstantVelocityMobilityModel>(); //获取CopterID对应的Nodes，例如1号飞机对应Node.get(0)
                mob->SetVelocity(Vector(data3d.VelE[0], data3d.VelE[1], 0));
                mob->SetPosition(Vector(data3d.PosE[0], data3d.PosE[1], 0));
                // Mobilitychange(mob, data3d.VelE[0], data3d.VelE[1], -data3d.VelE[2], data3d.PosE[0], data3d.PosE[1], -data3d.PosE[2]); //配置CopterID号飞机的位置，注意Z轴调整为向上为正
                cout << "Receive SOut2SimulatorSimpleTime data" << data3d.VelE[0] << data3d.VelE[1] << -data3d.VelE[2] << data3d.PosE[0] << data3d.PosE[1] << -data3d.PosE[2] << endl;
            }
            return;
        }
    }
}

