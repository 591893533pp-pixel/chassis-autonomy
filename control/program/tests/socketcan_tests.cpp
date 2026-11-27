// Exercise real SocketCAN serialization/RX with local datagrams, never a physical CAN bus.
#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#define private public
#include "elmue_can/elmue_can.h"
#undef private
#include <linux/can.h>
#include <linux/can/error.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <stdexcept>
static void require(bool b){if(!b)throw std::runtime_error("SocketCAN test failed");}
int main(){
 using namespace elmue_can;
 {ElmueCan bad;require(!bad.open("socketcan:no_such_can"));require(!bad.open("socketcan:"));}
 for(bool extended:{false,true}){
  int pair[2];require(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK,0,pair)==0);
  ElmueCan bus;bus.socket_fd_=pair[0];bus.started_=true;
  ElmueCanFrame in{};in.extended=extended;in.can_id=extended?0x12001f01:0x601;in.data_len=8;
  for(int i=0;i<8;++i)in.data[i]=i+1;
  require(bus.sendFrame(in));can_frame wire{};require(read(pair[1],&wire,sizeof(wire))==sizeof(wire));
  require(wire.can_id==(in.can_id|(extended?CAN_EFF_FLAG:0)));require(wire.can_dlc==8);require(!std::memcmp(wire.data,in.data,8));
  in.fd=true;require(!bus.sendFrame(in));in.fd=false;in.data_len=9;require(!bus.sendFrame(in));in.data_len=8;
  std::atomic<int> received{0};
  bus.setRxCallback([&](uint8_t type,const uint8_t*data,size_t len){
   constexpr size_t head=offsetof(kRxFrameElmue,timestamp);
   require(type==MSG_RxFrame&&len==head+8);auto*r=reinterpret_cast<const kRxFrameElmue*>(data);
   require(r->can_id==wire.can_id);require(!std::memcmp(data+head,wire.data,8));++received;
  });
  bus.rx_running_=true;bus.rx_thread_=std::thread(&ElmueCan::socketCanRxLoop,&bus);
  require(write(pair[1],&wire,sizeof(wire))==sizeof(wire));
  for(int i=0;i<100&&received==0;++i)std::this_thread::sleep_for(std::chrono::milliseconds(2));
  require(received==1);
  wire.can_id=CAN_ERR_FLAG|CAN_ERR_BUSOFF;require(write(pair[1],&wire,sizeof(wire))==sizeof(wire));
  for(int i=0;i<100&&!bus.socket_fault_;++i)std::this_thread::sleep_for(std::chrono::milliseconds(2));
  require(bus.socket_fault_);require(!bus.sendFrame(in));require(received==1);
  ElmueCanFrame stop{};stop.extended=true;stop.can_id=0x12001f01;stop.data_len=8;stop.data[0]=0x0a;stop.data[1]=0x70;
  require(bus.sendFrame(stop));require(read(pair[1],&wire,sizeof(wire))==sizeof(wire));
  stop.data[4]=1;require(!bus.sendFrame(stop));
  stop.can_id=0x04001f01;require(bus.sendFrame(stop));require(read(pair[1],&wire,sizeof(wire))==sizeof(wire));
  bus.close();require(bus.socket_fd_==-1&&!bus.started());close(pair[1]);
 }
}
