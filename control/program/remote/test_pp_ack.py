"""Compile the actual PP function against a fake SDO transport and virtual clock."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]

def test_pp_ack_failure_propagation(tmp_path):
 source=(ROOT/'third_party/elmue_can/src/zeroerr_canopen.cpp').read_text()
 function=source.split('bool ZeroErrCanopenDriver::moveToPositionPP',1)[1].split('bool ZeroErrCanopenDriver::haltPositionPP',1)[0]
 function='bool ZeroErrCanopenDriver::moveToPositionPP'+function
 # Replace only the clock for deterministic timeout testing; execute the source function itself.
 function=function.replace('steady_clock::now()', 'FakeClock::now()')
 stub=r'''
#include <optional>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cassert>
using namespace std::chrono;
struct FakeClock {static steady_clock::time_point now(){static int t=0;t+=100;return steady_clock::time_point(milliseconds(t));}};
void sleepMs(int){}
namespace canopen {
constexpr uint16_t SW_BIT_SETPOINT_ACK=4096,CW_ENABLE_OPERATION=15,CW_BIT_CHANGE_IMMEDIATELY=32,CW_BIT_ABS_REL=64,CW_BIT_NEW_SETPOINT=16,OD_TARGET_POSITION=0x607a;
struct ScaleConfig {int32_t radToCounts(double x)const{return int32_t(x);}};
}
struct ZeroErrCanopenDriver {
 int scenario=0;bool triggered=false;int trigger_count=0;
 canopen::ScaleConfig scaleFor(uint8_t){return {};}
 bool setProfileParams(uint8_t,double){return true;}
 struct Reply{bool success=true;};
 Reply sdoWrite(uint8_t,uint16_t,int,uint32_t,int){return {};}
 bool setControlword(uint8_t,uint16_t cw){triggered=cw&16;if(triggered)++trigger_count;return !(scenario==4&&!triggered&&trigger_count);}
 std::optional<uint16_t> readStatusword(uint8_t,double){
  if(scenario==1)return std::nullopt;
  if(scenario==2)return 4096;
  if(scenario==3)return 0;
  return triggered?4096:0;
 }
 bool moveToPositionPP(uint8_t,double,double,bool,bool);
};
'''
 main=r'''
int main(){
 for(int s=0;s<5;++s){ZeroErrCanopenDriver d;d.scenario=s;bool ok=d.moveToPositionPP(1,10,3,true,false);
 assert(ok==(s==0));if(s==1||s==2)assert(d.trigger_count==0);
 }
}
'''
 cpp=tmp_path/'pp.cpp';cpp.write_text(stub+function+main)
 binary=tmp_path/'pp'
 subprocess.run(['c++','-std=c++17',str(cpp),'-o',str(binary)],check=True,capture_output=True)
 subprocess.run([str(binary)],check=True,capture_output=True)
