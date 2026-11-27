// Bounded suspended-wheel identification. One nonzero wheel target, no lift.
#include "auxiliary.hpp"
#include <chrono>
#include <thread>
#include <csignal>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
static volatile sig_atomic_t running=1;
static void stop(int){running=0;}
static void check(poke::Status s){if(!s)throw std::runtime_error(s.message());}
int main(int argc,char**argv){
 if(argc!=3&&argc!=5){std::cerr<<"Usage: single_wheel SIGNED_ID LOG [--hardware can0] (negative ID reverses direction)\n";return 2;}
 int id=0;try{std::string v=argv[1];size_t n;id=std::stoi(v,&n);if(n!=v.size())return 2;}catch(...){return 2;}
 if(id < -4||id>4 || (argc==5&&std::string(argv[3])!="--hardware"))return 2;
 const double direction=id<0?-1.:1.;id=std::abs(id);
 bool hardware=argc==5;
 std::string lockpath="/tmp/poke-remote-"+std::to_string(getuid())+".lock";
 int lock=open(lockpath.c_str(),O_CREAT|O_WRONLY,0600);
 if(lock<0||flock(lock,LOCK_EX|LOCK_NB)){std::cerr<<"Another CAN session owns lock\n";return 2;}
 signal(SIGINT,stop);signal(SIGTERM,stop);
 std::ofstream log(argv[2]);if(!log)return 2;
 log<<std::setprecision(12)<<"elapsed_s,hardware,selected_id,target_rad_s,valid,age_ms";
 for(int i=1;i<=4;i++)log<<",wheel"<<i<<"_rad_s,wheel"<<i<<"_fault";
 log<<'\n';
 poke::AuxiliaryConfig c;c.serial=hardware?"socketcan:"+std::string(argv[4]):"";
 c.enable_chassis=true;c.enable_lift=false;c.enable_on_robot_enable=false;
 c.chassis_speed_rad_s=.2;c.chassis_feedback_timeout_ms=100;
 poke::detail::AuxiliaryController device(c);int result=0;
 try{
  if(hardware){check(device.connect());if(!running)throw std::runtime_error("interrupted");check(device.setChassisEnabled(true));}
  const double duration=10.;
  std::cerr<<"ID "<<id<<": "<<(id==0?0.:direction*.2)<<" rad/s maximum, "<<duration<<" seconds then zero; ID 0 is all-zero feedback test. "<<(hardware?"HARDWARE":"DRY RUN")<<'\n';
  auto start=std::chrono::steady_clock::now();
  while(running){
   double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
   if(t>=duration+.5)break;
   double speed=t<duration? direction*.2*std::min(t/.5,1.):0.;
   std::array<double,4> target{};if(id>0)target[id-1]=speed;else speed=0;
   poke::AuxiliaryState s;
   if(hardware){check(device.refreshChassisFeedback());s=device.state();if(!s.chassis_velocity_feedback_valid)throw std::runtime_error("invalid feedback");check(device.setChassisWheelVelocities(target));}
   log<<t<<','<<hardware<<','<<id<<','<<speed<<','<<s.chassis_velocity_feedback_valid<<','<<s.chassis_velocity_feedback_max_age_ms;
   for(int i=0;i<4;i++)log<<','<<s.chassis_velocities[i]<<','<<int(s.chassis_fault_codes[i]);
   log<<'\n';log.flush();if(!log)throw std::runtime_error("log write failed");
   std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
 }catch(const std::exception&e){std::cerr<<"FAULT "<<e.what()<<'\n';result=1;}
 if(hardware&&device.state().connected){
  auto s=device.stopChassis();if(!s){std::cerr<<"STOP FAILED "<<s.message()<<'\n';result=1;}
  s=device.disable();if(!s){std::cerr<<"DISABLE FAILED "<<s.message()<<'\n';result=1;}
  device.close();
 }
 std::cerr<<"Finished; "<<(hardware?"stop/disable attempted":"no hardware accessed")<<"; exit="<<result<<'\n';
 close(lock);return result;
}
