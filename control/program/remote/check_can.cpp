// Read-only transport preflight: binds socket and reads link config, sends no CAN frames.
#include "elmue_can/elmue_can.h"
#include <iostream>
int main(int argc,char**argv){
 elmue_can::ElmueCan bus;
 std::string name=argc>1?argv[1]:"can0";
 if(!bus.open("socketcan:"+name)||!bus.setBitrateHz(1000000)){
  std::cerr<<bus.lastError()<<'\n';return 1;
 }
 std::cout<<name<<": SocketCAN socket accessible; UP, classic CAN, 1 Mbps, normal mode. No CAN frames sent; motor communication not verified.\n";
}
