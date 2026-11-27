#pragma once
#include "poke/chassis_kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
// Segment-local wheel odometry, not absolute localization. Stop always bypasses shaping.
struct BodyControl {
 double linear_accel;
 explicit BodyControl(double acceleration=.10):linear_accel(acceleration){
  if(!std::isfinite(acceleration)||acceleration<=0)throw std::runtime_error("invalid linear acceleration");
 }
 poke::ChassisTwist shaped{}, filtered{};
 double x=0,y=0,yaw=0,dx=0,dy=0,cross_error=0,yaw_trim=0,cross_trim=0;
 double planar_correction=0;
 bool planar_limited=false;
 bool holding=false,initialized=false;
 void reset(){*this=BodyControl(linear_accel);}
 static double dead(double v,double band){return std::abs(v)<=band?0:std::copysign(std::abs(v)-band,v);}
 poke::ChassisTwist update(poke::ChassisTwist request,poke::ChassisTwist measured,double dt,bool fresh,bool enable_hold=true){
  for(double v:{request.vx_m_s,request.vy_m_s,request.omega_z_rad_s,measured.vx_m_s,measured.vy_m_s,measured.omega_z_rad_s})
   if(!std::isfinite(v)){reset();throw std::runtime_error("nonfinite body control input");}
  const double speed=std::hypot(request.vx_m_s,request.vy_m_s);
  if(speed<1e-8&&std::abs(request.omega_z_rad_s)<1e-8){reset();return {};}
  if(!fresh){reset();throw std::runtime_error("body control requires fresh wheel feedback");}
  if(!std::isfinite(dt)||dt<=0||dt>.1){reset();return {};}
  const bool straight=enable_hold&&speed>1e-8&&std::abs(request.omega_z_rad_s)<1e-8;
  const double nx=straight?request.vx_m_s/speed:0,ny=straight?request.vy_m_s/speed:0;
  // New segment on release, turn, or direction change; never drive back to an old waypoint.
  const bool new_segment=straight&&(!holding||nx*dx+ny*dy<.999);
  if(new_segment){x=y=yaw=0;dx=nx;dy=ny;filtered=measured;initialized=true;}
  if(!initialized){filtered=measured;initialized=true;}
  const double a=dt/(.08+dt);
  filtered.vx_m_s+=a*(measured.vx_m_s-filtered.vx_m_s);
  filtered.vy_m_s+=a*(measured.vy_m_s-filtered.vy_m_s);
  filtered.omega_z_rad_s+=a*(measured.omega_z_rad_s-filtered.omega_z_rad_s);
  holding=straight;cross_trim=yaw_trim=cross_error=planar_correction=0;planar_limited=false;
  auto desired=request;
  if(straight){
   if(!new_segment){
    const double mid=yaw+.5*filtered.omega_z_rad_s*dt;
    x+=(std::cos(mid)*filtered.vx_m_s-std::sin(mid)*filtered.vy_m_s)*dt;
    y+=(std::sin(mid)*filtered.vx_m_s+std::cos(mid)*filtered.vy_m_s)*dt;
    yaw+=filtered.omega_z_rad_s*dt;
   }
   cross_error=-dy*x+dx*y;
   // At 5 cm/s a 0.02 rad/s yaw correction is too large and causes visible
   // hunting.  Keep the normal correction for faster motion, but use a
   // crawl-speed cap and a wider deadband below 8 cm/s.
   const bool crawl=speed<.08;
   const double cross_cap=crawl?.002:.005;
   const double yaw_cap=crawl?.006:.02;
   const double cross_deadband=crawl?.002:.001;
   const double yaw_deadband=crawl?.006:.003;
   cross_trim=std::clamp(-dead(cross_error,cross_deadband),-cross_cap,cross_cap);
   yaw_trim=std::clamp(-dead(yaw,yaw_deadband)-.10*dead(filtered.omega_z_rad_s,.008),-yaw_cap,yaw_cap);
   // Keep requested direction fixed in the segment's initial heading.
   const double wx=dx*speed-dy*cross_trim,wy=dy*speed+dx*cross_trim;
   desired.vx_m_s=std::cos(yaw)*wx+std::sin(yaw)*wy;
   desired.vy_m_s=-std::sin(yaw)*wx+std::cos(yaw)*wy;
   const double norm=std::hypot(desired.vx_m_s,desired.vy_m_s);
   if(norm>speed){desired.vx_m_s*=speed/norm;desired.vy_m_s*=speed/norm;}
   // Bound the whole hold correction, including the heading rotation above.
   // Bounding cross_trim alone permits integrated odometry bias to create
   // arbitrarily large sideways commands. Interpolation toward the request
   // also preserves the requested speed bound (a convex velocity disk).
   const double cx=desired.vx_m_s-request.vx_m_s,cy=desired.vy_m_s-request.vy_m_s;
   planar_correction=std::hypot(cx,cy);
   if(planar_correction>cross_cap){
    desired.vx_m_s=request.vx_m_s+cx*cross_cap/planar_correction;
    desired.vy_m_s=request.vy_m_s+cy*cross_cap/planar_correction;
    planar_correction=cross_cap;planar_limited=true;
   }
   desired.omega_z_rad_s=yaw_trim;
  }else{x=y=yaw=0;}
  // Vector acceleration limit preserves diagonal direction. Angular acceleration limited independently.
  double vx=desired.vx_m_s-shaped.vx_m_s,vy=desired.vy_m_s-shaped.vy_m_s;
  const double delta=std::hypot(vx,vy),cap=linear_accel*dt;
  if(delta>cap){vx*=cap/delta;vy*=cap/delta;}
  shaped.vx_m_s+=vx;shaped.vy_m_s+=vy;
  shaped.omega_z_rad_s+=std::clamp(desired.omega_z_rad_s-shaped.omega_z_rad_s,-.30*dt,.30*dt);
  return shaped;
 }
};
