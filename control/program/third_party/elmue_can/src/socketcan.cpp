// SocketCAN transport for classic 1 Mbps CAN. Keeps existing protocol decoders.
#include "elmue_can/elmue_can.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <cstring>
#include <array>
#include <cstdio>

namespace elmue_can {
bool ElmueCan::openSocketCan(const std::string& name) {
 if(name.empty()||name.size()>=IFNAMSIZ){last_error_="invalid SocketCAN interface";return false;}
 socket_fd_=socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,CAN_RAW);
 if(socket_fd_<0){last_error_=std::strerror(errno);return false;}
 ifreq req{};std::strncpy(req.ifr_name,name.c_str(),IFNAMSIZ-1);
 if(ioctl(socket_fd_,SIOCGIFINDEX,&req)<0){last_error_="SocketCAN interface not found: "+name;::close(socket_fd_);socket_fd_=-1;return false;}
 socket_index_=req.ifr_ifindex;
 sockaddr_can address{};address.can_family=AF_CAN;address.can_ifindex=socket_index_;
 can_err_mask_t errors=CAN_ERR_MASK;
 if(setsockopt(socket_fd_,SOL_CAN_RAW,CAN_RAW_ERR_FILTER,&errors,sizeof(errors))<0||
    bind(socket_fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0){
  last_error_=std::strerror(errno);::close(socket_fd_);socket_fd_=-1;return false;
 }
 serial_="socketcan:"+name; supports_fd_=false; supports_termination_=false;socket_fault_=false;
 return true;
}

bool ElmueCan::checkSocketCanBitrate(uint32_t baud) {
 // Read kernel link attributes; never reconfigure an active interface behind the operator.
 int fd=socket(AF_NETLINK,SOCK_RAW|SOCK_CLOEXEC,NETLINK_ROUTE);
 if(fd<0){last_error_="cannot query SocketCAN link";return false;}
 timeval timeout{1,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
 struct {nlmsghdr h; ifinfomsg i;} req{};
 req.h.nlmsg_len=NLMSG_LENGTH(sizeof(ifinfomsg));req.h.nlmsg_type=RTM_GETLINK;req.h.nlmsg_flags=NLM_F_REQUEST;req.h.nlmsg_seq=1;req.i.ifi_index=socket_index_;
 sockaddr_nl kernel{};kernel.nl_family=AF_NETLINK;
 int sent=sendto(fd,&req,req.h.nlmsg_len,0,reinterpret_cast<sockaddr*>(&kernel),sizeof(kernel));
 alignas(nlmsghdr) char buf[8192];int n=sent<0?-1:recv(fd,buf,sizeof(buf),0);::close(fd);
 bool up=false,classic=false;uint32_t actual=0;bool prohibited=false;
 for(auto*h=reinterpret_cast<nlmsghdr*>(buf);n>0&&NLMSG_OK(h,n);h=NLMSG_NEXT(h,n)){
  if(h->nlmsg_type!=RTM_NEWLINK)continue;
  auto*i=reinterpret_cast<ifinfomsg*>(NLMSG_DATA(h));if(i->ifi_index!=socket_index_)continue;
  up=i->ifi_flags&IFF_UP;int len=IFLA_PAYLOAD(h);
  for(auto*a=IFLA_RTA(i);RTA_OK(a,len);a=RTA_NEXT(a,len)){
   if(a->rta_type==IFLA_MTU&&RTA_PAYLOAD(a)>=4){uint32_t mtu;std::memcpy(&mtu,RTA_DATA(a),4);classic=mtu==CAN_MTU;}
   if(a->rta_type!=IFLA_LINKINFO)continue;
   int l=RTA_PAYLOAD(a);
   for(auto*b=reinterpret_cast<rtattr*>(RTA_DATA(a));RTA_OK(b,l);b=RTA_NEXT(b,l)){
    if(b->rta_type!=IFLA_INFO_DATA)continue;int k=RTA_PAYLOAD(b);
    for(auto*c=reinterpret_cast<rtattr*>(RTA_DATA(b));RTA_OK(c,k);c=RTA_NEXT(c,k)){
     if(c->rta_type==IFLA_CAN_BITTIMING&&RTA_PAYLOAD(c)>=sizeof(can_bittiming)){can_bittiming t{};std::memcpy(&t,RTA_DATA(c),sizeof(t));actual=t.bitrate;}
     if(c->rta_type==IFLA_CAN_CTRLMODE&&RTA_PAYLOAD(c)>=sizeof(can_ctrlmode)){can_ctrlmode m{};std::memcpy(&m,RTA_DATA(c),sizeof(m));prohibited=m.flags&(CAN_CTRLMODE_LISTENONLY|CAN_CTRLMODE_LOOPBACK|CAN_CTRLMODE_FD);}
    }
   }
  }
 }
 if(!up||!classic||actual!=baud||prohibited){last_error_="SocketCAN must be UP, classic CAN, bitrate="+std::to_string(baud)+", normal mode; configure with setup_can.sh";return false;}
 return true;
}

bool ElmueCan::sendSocketCan(const ElmueCanFrame& frame) {
 // A latched fault blocks motion but must not suppress explicit stop/disable attempts.
 const unsigned type=frame.can_id>>24;
 const bool zero_velocity=frame.extended && type==18 && frame.data_len==8 &&
     frame.data[0]==0x0a && frame.data[1]==0x70 &&
     frame.data[4]==0 && frame.data[5]==0 && frame.data[6]==0 && (frame.data[7]&0x7f)==0;
 const bool motor_disable=frame.extended && type==4 && frame.data_len==8;
 const bool lift_stop=!frame.extended && frame.can_id>=0x601 && frame.can_id<=0x67f &&
     frame.data_len==8 && frame.data[0]==0x2b && frame.data[1]==0x40 && frame.data[2]==0x60 && frame.data[3]==0 &&
     ((frame.data[4]==0x0f && frame.data[5]==1) || (frame.data[4]==6 && frame.data[5]==0) || (frame.data[4]==0 && frame.data[5]==0));
 if(!started_||(socket_fault_ && !(zero_velocity||motor_disable||lift_stop))||frame.fd||frame.brs||frame.data_len>8||frame.can_id>(frame.extended?CAN_EFF_MASK:CAN_SFF_MASK)){
  last_error_="SocketCAN invalid frame/state or latched bus fault";return false;
 }
 can_frame f{};f.can_id=frame.can_id|(frame.extended?CAN_EFF_FLAG:0)|(frame.rtr?CAN_RTR_FLAG:0);f.can_dlc=frame.data_len;
 std::memcpy(f.data,frame.data,frame.data_len);
 // Nonblocking: queue-full is an explicit failure, never wait indefinitely while moving.
 if(write(socket_fd_,&f,sizeof(f))!=sizeof(f)){++tx_errors_;last_error_=std::strerror(errno);return false;}
 ++tx_count_;return true;
}

void ElmueCan::socketCanRxLoop(){
 while(rx_running_){
  pollfd p{socket_fd_,POLLIN,0};int ready=poll(&p,1,50);
  if(ready<0&&errno==EINTR)continue;
  if(ready<0||(p.revents&(POLLERR|POLLHUP|POLLNVAL))){socket_fault_=true;++rx_errors_;break;}
  if(!ready)continue;
  can_frame f{};ssize_t n=read(socket_fd_,&f,sizeof(f));
  if(n<0&&(errno==EAGAIN||errno==EINTR))continue;
  if(n!=sizeof(f)||f.can_dlc>8){socket_fault_=true;++rx_errors_;break;}
  if(f.can_id&CAN_ERR_FLAG){
   std::fprintf(stderr,"SocketCAN error id=0x%08x data=",f.can_id);
   for(unsigned i=0;i<f.can_dlc;++i)std::fprintf(stderr,"%02x ",f.data[i]);
   std::fprintf(stderr,"; fault latched, only stop/disable writes permitted\n");
   socket_fault_=true;++rx_errors_;continue;
  }
  if(f.can_id&CAN_RTR_FLAG)continue;
  ++rx_usb_count_;
  constexpr size_t header=offsetof(kRxFrameElmue,timestamp);
  std::array<uint8_t,header+8> payload{};
  auto*r=reinterpret_cast<kRxFrameElmue*>(payload.data());
  r->header.msg_type=MSG_RxFrame;r->header.size=header+f.can_dlc;r->can_id=f.can_id;
  std::memcpy(payload.data()+header,f.data,f.can_dlc);
  if(rx_callback_)rx_callback_(MSG_RxFrame,payload.data(),header+f.can_dlc);
 }
}
} // namespace elmue_can
