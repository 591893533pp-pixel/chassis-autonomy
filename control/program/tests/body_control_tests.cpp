#include "body_control.hpp"
#include <stdexcept>
void check(bool b){if(!b)throw std::runtime_error("body control test failed");}
int main(){
 BodyControl baseline;
 for(int i=0;i<500;++i){auto u=baseline.update({.05,0,0},{.04,.02,(i%2?.1:-.1)},.02,true,false);
 check(u.vy_m_s==0&&u.omega_z_rad_s==0&&!baseline.holding);}
 auto stopped=baseline.update({}, {},.02,true,false);check(stopped.vx_m_s==0);
 BodyControl c;poke::ChassisTwist forward{.02,0,0},measurement{};
 auto v=c.update(forward,measurement,.02,true);check(std::abs(v.vx_m_s-.002)<1e-9);
 for(int i=0;i<20;++i)v=c.update(forward,measurement,.02,true);
 check(std::abs(v.vx_m_s-.02)<1e-9&&v.vy_m_s==0&&v.omega_z_rad_s==0);
 for(int i=0;i<200;++i)v=c.update(forward,{.02,.01,.02},.02,true);
 check(c.cross_error>0&&c.cross_trim<0&&c.yaw_trim<0);
 check(std::abs(c.cross_trim)<=.005&&std::abs(c.yaw_trim)<=.02);
 check(std::hypot(v.vx_m_s,v.vy_m_s)<=.020001);
 v=c.update({},measurement,.02,false);check(v.vx_m_s==0&&v.vy_m_s==0&&v.omega_z_rad_s==0&&!c.holding&&c.yaw==0);
 v=c.update({0,0,.04},{},.02,true);check(!c.holding&&v.omega_z_rad_s>0&&v.omega_z_rad_s<=.006001);
 v=c.update(forward,{},.02,true);check(c.holding&&c.cross_error==0);
 v=c.update({0,.02,0},{},.02,true);check(c.cross_error==0&&c.yaw==0&&c.dx==0&&c.dy==1);
 v=c.update(forward,{},.3,true);check(v.vx_m_s==0&&!c.holding);
 bool threw=false;try{c.update(forward,{},.02,false);}catch(...){threw=true;}check(threw);
 // Closed-loop simulated lateral disturbance: segment error stays bounded rather than accumulating freely.
 c.reset();measurement={};double lateral=0;
 for(int i=0;i<500;++i){v=c.update(forward,measurement,.02,true);measurement={v.vx_m_s,v.vy_m_s+.002,v.omega_z_rad_s};lateral+=measurement.vy_m_s*.02;}
 check(std::abs(lateral)<.006); // Open-loop disturbance would yield 0.020 m over ten seconds.
 // A biased wheel-odometry heading must not bypass the planar correction
 // budget via the segment-to-body rotation. Exercise every translation axis
 // and both bias signs; this is a software bound, not a motor/traction model.
 for(double speed:{.05,.09})for(double angle:{0.,1.5707963267948966,3.141592653589793,0.7853981633974483})for(double bias:{-.02,.02}){
  c.reset();poke::ChassisTwist request{speed*std::cos(angle),speed*std::sin(angle),0};
  const double cap=speed<.08?.002:.005;
  for(int i=0;i<1500;++i){
   v=c.update(request,{request.vx_m_s,request.vy_m_s,bias},.02,true);
   if(i>100)check(std::hypot(v.vx_m_s-request.vx_m_s,v.vy_m_s-request.vy_m_s)<=cap+1e-9);
   check(std::hypot(v.vx_m_s,v.vy_m_s)<=speed+1e-9);
  }
  v=c.update({}, {},.02,false);check(v.vx_m_s==0&&v.vy_m_s==0&&v.omega_z_rad_s==0);
 }
 // Remote profile: 0.2 m/s^2 ramp, reset preserves configured acceleration.
 BodyControl faster(.2);double distance=0;
 for(int k=0;k<275;++k){
  double t=k*.02;
  double target=t<3?.5:std::max(0.,.5-.2*(t-3));
  auto u=faster.update({target,0,0},{},.02,true,false);
  distance+=u.vx_m_s*.02;
  if(k==49)check(std::abs(u.vx_m_s-.2)<1e-9);
  check(u.vy_m_s==0&&u.omega_z_rad_s==0&&u.vx_m_s<=.500001);
 }
 check(distance>1.45&&distance<1.55);
 auto zero=faster.update({}, {},.02,false,false);check(zero.vx_m_s==0);
 auto restart=faster.update({.5,0,0},{},.02,true,false);check(std::abs(restart.vx_m_s-.004)<1e-9);
}
