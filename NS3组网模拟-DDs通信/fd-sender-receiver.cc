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
uint16_t copterID = 1;

void receivePacket(Ptr<Socket> sock);
void receivePacketFromNx(Ptr<Socket> sock);
void receiveMulicastPacketFromNx(Ptr<Socket> sock);
void ReceiveWin(Ptr<Node> nodelist[], int uavNum, string context, const Ptr<const Packet> packet);


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

    UdpEchoServerHelper echosever_Win(19000);
    ApplicationContainer severapp_Win;
    severapp_Win = echosever_Win.Install(receiver.Get(0));
    severapp_Win.Start(Seconds(1.0));
    severapp_Win.Stop(Seconds(500.0));  

    //Here is  an uav network
    uavs.Create(uav);


    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");
    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange", DoubleValue(200));
    phy.SetChannel(channel.Create());
    WifiMacHelper mac;
    Ssid ssid;
    mac.SetType("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer devices = wifi.Install(phy, mac, uavs);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(50),
                                  "DeltaY", DoubleValue(50),
                                  "GridWidth", UintegerValue(uav),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(uavs);

    InternetStackHelper stackUavs;
    Ipv4ListRoutingHelper list;
    OlsrHelper olsr;
    list.Add(olsr,10);
    stackUavs.SetRoutingHelper(list);
    stackUavs.Install(uavs);

    Ipv4AddressHelper addressUavs;
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


    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
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
    // sourece_address.GetIpv4();
    // cout << "source : " << sourece_address.GetIpv4() << "  :  "<< sourece_address.GetPort() << endl;
    packet->CopyData((uint8_t *)&dest_ip_port, 8);
    Ipv4Address temp = temp.Deserialize(dest_ip_port.ip);
    if(!IpIdMap[sourece_address.GetIpv4()]){
        if(int(copterID) > uav) return ;
        IpIdMap[sourece_address.GetIpv4()] = copterID;
        copterID++;
    }



    // InetSocketAddress addr2 = InetSocketAddress("192.168.31.188", uint16_t(dest_ip_port.port));
    // cout << "dest   : "<< temp.Deserialize(dest_ip_port.ip) << "  : "<< uint16_t(dest_ip_port.port)<< endl;
    //send_sock->Bind(sourece_address);
    // cout << send_sock->Connect(addr2) << endl;
    //cout << addr2 << endl;
    // packet->RemoveAtStart(8);
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    // InetSocketAddress dest_addr = InetSocketAddress(temp.Deserialize(dest_ip_port.ip), uint16_t(dest_ip_port.port)) ;
    if(!IpIdMap[temp]){
        // return ;
        if(int(copterID) > uav) return ;
        IpIdMap[temp] = copterID;
        copterID++;
    }
    Ptr<Socket> sender = Socket::CreateSocket(uavs.Get(IpIdMap[sourece_address.GetIpv4()]-1), tid);
    Ptr<Ipv4> ippp = uavs.Get(IpIdMap[temp]-1)-> GetObject<Ipv4> ();
    temp = ippp->GetAddress(1,0).GetLocal();
    // cout << temp << endl;
    InetSocketAddress dest_addr = InetSocketAddress(temp, 20000);
    sender->Bind();
    sender->Connect(dest_addr);
    sender->Send(packet);
    // cout <<  << endl;
    sender->Close();
}


void receiveMulicastPacketFromNx(Ptr<Socket> sock){
    Address from;
    Ptr<Packet> packet = sock->RecvFrom(from);
    InetSocketAddress sourece_address = InetSocketAddress::ConvertFrom(from);
    packet->CopyData((uint8_t *)&dest_ip_port, 16);
    packet->RemoveAtStart(16);
    Ipv4Address temp;
    temp = temp.Deserialize(dest_ip_port.ip);
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sender = Socket::CreateSocket(receiver.Get(0), tid);
    InetSocketAddress dest_addr = InetSocketAddress("255.255.255.255", dest_ip_port.port);
    // InetSocketAddress source_addr_in_ns3 = InetSocketAddress(ippp->GetAddress(1,0).GetLocal(), 10000);
    // cout << "source : " << sourece_address.GetIpv4() <<"  : " << sourece_address.GetPort() <<'\n';
    sender->SetAllowBroadcast(true);
    sender->Bind(sourece_address);
    sender->Connect(dest_addr);
    sender->Send(packet);
    sender->Close();
}


void ReceiveWin(Ptr<Node> nodelist[], int uavNum, string context, const Ptr<const Packet> packet)
{
    //cout << "received*******************" << endl;
    //return ;
    //haven't deconde the packet to get the locations of the uavs.
    //uint8_t copterID[8];
    packet->CopyData((uint8_t *)&dest_ip_port, 8);
    // for(int i=0; i<4; i++){
    //     cout << int(dest_ip_port.ip[i]) <<".";
    // }
    // cout << "  :   "  << dest_ip_port.port << endl;


    // TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    // Ptr<Socket> send_sock = Socket::CreateSocket(nodelist[source], tid);
    // InetSocketAddress addr = InetSocketAddress(nodelist[dest]
    //         ->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(), 10000);
    // send_sock->Connect(addr);
    // Ptr<Packet> p = packet->Copy();
    // send_sock->Send(p);

    Ptr<Packet> packet1 = packet->Copy();
    cout << packet1->ToString() << endl;

    // PppHeader ppp ;
    // // packet1->RemoveHeader(ppp);
    // Ipv4Header ip;
    // packet1->RemoveHeader(ip);
    // cout << "Source IP:  " << ip.GetSource() <<endl;



    // cout << "reveived  " << receivenum++<< endl;
    // uint8_t data[11];
    // packet->CopyData((uint8_t *)&data, 11);
    // cout << data[0] << endl;
    // for(int i=0; i<11; i++){
    //     cout << char(data[i]);
    // }
    // cout << endl;

    // TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    // Ptr<Socket> send_sock = Socket::CreateSocket(nodelist[0], tid);
    // InetSocketAddress addr = InetSocketAddress(Ipv4Address("192.168.31.71"), 30005);
    // send_sock->Connect(addr);
    // uint8_t datap[sizeof(packet)];
    // packet->CopyData(datap,sizeof(datap));
    // Ptr< Packet> p =Create<Packet>(datap,sizeof(datap));
    // send_sock->Send(p);
}
