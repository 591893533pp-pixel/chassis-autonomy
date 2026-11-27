// Single CAN owner. No hardware access unless explicitly given --hardware SERIAL.
#include "auxiliary.hpp"
#include "wheel_balance.hpp"
#include "body_control.hpp"
#include "poke/chassis_kinematics.hpp"
#include <algorithm>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <time.h>
static volatile sig_atomic_t running=1;
static void stop_signal(int) { running=0; }
static double now() { timespec t{}; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec*1e-9; }
static void check(poke::Status s) { if(!s) throw std::runtime_error(s.message()); }
struct Command { double stamp=0,x=0,y=0,yaw=0,lift=0; int enabled=0; };
static Command parse(const std::string& line) {
 Command c; std::istringstream in(line); std::string extra;
 if(!(in>>c.stamp>>c.enabled>>c.x>>c.y>>c.yaw>>c.lift) || (in>>extra) ||
    !std::isfinite(c.stamp)||!std::isfinite(c.x)||!std::isfinite(c.y)||!std::isfinite(c.yaw)||!std::isfinite(c.lift)||
    (c.enabled!=0&&c.enabled!=1)||std::hypot(c.x,c.y)>1.500001||std::abs(c.yaw)>0.400001||std::abs(c.lift)>1)
   throw std::runtime_error("invalid command frame");
 return c;
}
int main(int argc,char**argv) {
 bool hardware=false,lift=false,body_hold=true,wheel_pi=true,log_every_cycle=true; std::string serial,log;
 double crawl_kp=NAN,crawl_ki=NAN,normal_kp=NAN,normal_ki=NAN;
 WheelBalance::Profile pi_profile=WheelBalance::Profile::Baseline;
 for(int i=1;i<argc;++i) { std::string a=argv[i];
  if(a=="--hardware"&&i+1<argc){hardware=true;serial=argv[++i];}
  else if(a=="--lift") lift=true;
  else if(a=="--diagnostic-no-body-hold") body_hold=false;
  else if(a=="--diagnostic-no-wheel-pi") wheel_pi=false;
  else if(a=="--log-every-cycle") log_every_cycle=true;
  else if(a=="--wheel-pi-profile"&&i+1<argc) {
   std::string value=argv[++i];
   if(value=="baseline")pi_profile=WheelBalance::Profile::Baseline;
   else if(value=="smooth-candidate")pi_profile=WheelBalance::Profile::SmoothCandidate;
   else if(value=="integral-gate")pi_profile=WheelBalance::Profile::IntegralGate;
   else {std::cerr<<"invalid wheel PI profile\n";return 2;}
  }
  else if(a=="--log"&&i+1<argc) log=argv[++i];
  else if(a=="--crawl-kp"&&i+1<argc) crawl_kp=std::stod(argv[++i]);
  else if(a=="--crawl-ki"&&i+1<argc) crawl_ki=std::stod(argv[++i]);
  else if(a=="--normal-kp"&&i+1<argc) normal_kp=std::stod(argv[++i]);
  else if(a=="--normal-ki"&&i+1<argc) normal_ki=std::stod(argv[++i]);
  else {std::cerr<<"Usage: remote_bridge [--hardware SERIAL] [--lift] [--diagnostic-no-body-hold] [--diagnostic-no-wheel-pi] [--wheel-pi-profile baseline|smooth-candidate|integral-gate] [--crawl-kp K] [--crawl-ki I] [--normal-kp K] [--normal-ki I] [--log-every-cycle] --log FILE\n";return 2;}
 }
 for(double v:{crawl_kp,crawl_ki,normal_kp,normal_ki}) if(std::isfinite(v)&&(v<0||v>100)) {std::cerr<<"invalid wheel PI gain\n";return 2;}
 if(log.empty()) return 2;
 std::ofstream out(log); if(!out) return 2;
 out<<std::setprecision(12);
 out<<"monotonic_s,hardware,enabled,command_age_s,requested_vx,requested_vy,requested_wz,lift_request,applied_vx,applied_vy,applied_wz,feedback_valid,feedback_age_ms";
 for(int i=1;i<=4;++i) out<<",wheel"<<i<<"_cmd_rad_s,wheel"<<i<<"_feedback_rad_s,wheel"<<i<<"_temp_C,wheel"<<i<<"_fault";
 out<<",feedback_vx,feedback_vy,feedback_wz,lift_enabled,lift_position_valid,lift_feedback_age_s,lift_mm,lift_mm_s,lift_target_mm,lift_statusword,lift_error_cached,lift_lower_software,lift_upper_software,loop_ms,missed_control_slots";
 for(int i=1;i<=4;++i)out<<",wheel"<<i<<"_nominal_rad_s,wheel"<<i<<"_trim_rad_s,wheel"<<i<<"_filtered_rad_s";
 out<<",wheel_balance_active,body_control_active,body_hold_active,segment_x_m,segment_y_m,segment_yaw_rad,cross_error_m,cross_trim_m_s,yaw_trim_rad_s,shaped_vx,shaped_vy,shaped_wz,body_hold_planar_correction_m_s,body_hold_planar_limited,wheel1_filtered_target_rad_s,wheel2_filtered_target_rad_s,wheel3_filtered_target_rad_s,wheel4_filtered_target_rad_s,wheel_pi_profile,pi_crawl_kp,pi_crawl_ki,pi_normal_kp,pi_normal_ki";
 for(int i=1;i<=4;++i)out<<",wheel"<<i<<"_p_term_rad_s,wheel"<<i<<"_i_term_rad_s,wheel"<<i<<"_integral_enabled,wheel"<<i<<"_stable_s,wheel"<<i<<"_target_rate_rad_s2";
 out<<'\n';
 signal(SIGINT,stop_signal);signal(SIGTERM,stop_signal);signal(SIGPIPE,SIG_IGN);
 fcntl(STDIN_FILENO,F_SETFL,fcntl(STDIN_FILENO,F_GETFL)|O_NONBLOCK);
 std::unique_ptr<poke::detail::AuxiliaryController> device;
 // 1.5 m/s + 0.4 rad/s yaw requires up to 21.01 rad/s; 25 is a software cap, not hardware qualification.
 poke::XDriveKinematics kin(.0762,.25213,25.0);
 WheelBalance balance(25.0,pi_profile,crawl_kp,crawl_ki,normal_kp,normal_ki); BodyControl body(1.0);
 const double actual_crawl_kp=std::isfinite(crawl_kp)?crawl_kp:.12;
 const double actual_crawl_ki=std::isfinite(crawl_ki)?crawl_ki:.12;
 const double actual_normal_kp=std::isfinite(normal_kp)?normal_kp:(pi_profile==WheelBalance::Profile::SmoothCandidate?.12:.20);
 const double actual_normal_ki=std::isfinite(normal_ki)?normal_ki:(pi_profile==WheelBalance::Profile::SmoothCandidate?.12:.30);
 const char* profile_name=pi_profile==WheelBalance::Profile::Baseline?"baseline":
                          pi_profile==WheelBalance::Profile::SmoothCandidate?"smooth-candidate":"integral-gate";
 bool powered=false,lift_moving=false; double lift_read=0,last_log=0,last_loop=now(); int result=0;
 auto cleanup=[&] {
  if(device && device->state().connected) {
   auto s=device->stopChassis(); if(!s)std::cerr<<"STOP CHASSIS FAILED: "<<s.message()<<'\n';
   if(lift){s=device->stopLift();if(!s)std::cerr<<"STOP LIFT FAILED: "<<s.message()<<'\n';}
   s=device->disable();if(!s)std::cerr<<"DISABLE FAILED: "<<s.message()<<'\n'; device->close();
  }
 };
 try {
  if(hardware) {
   poke::AuxiliaryConfig c; c.serial=serial;c.enable_chassis=true;c.enable_lift=lift;c.enable_on_robot_enable=false;
   c.chassis_speed_rad_s=25;c.chassis_feedback_timeout_ms=100;
   c.lift_control_mode=poke::LiftControlMode::MetricProfilePosition;
   c.lift_travel_counts=0; // Metric PP uses absolute 0–250 mm, not the raw-count session guard.
   c.lift_motor_counts_per_revolution=800;c.lift_reduction_ratio=7;c.lift_screw_lead_mm_per_revolution=48;
   c.lift_min_position_mm=0;c.lift_max_position_mm=250;c.lift_max_profile_velocity_mm_s=3;
   device=std::make_unique<poke::detail::AuxiliaryController>(c);check(device->connect());
  }
  std::cerr<<"Bridge mapping: physical vx/vy -> legacy -vx/-vy; yaw unchanged; revision final_front_20260917\n"
           <<"Low-speed bounded profile: motor velocity mode + filtered wheel PI (crawl kp="<<actual_crawl_kp<<" ki="<<actual_crawl_ki<<" trim<=15%, matched target/feedback tau=120ms)\n"
           <<"Body hold "<<(body_hold?"ON":"OFF (diagnostic)")<<"; acceleration shaping retained; stop overrides\n"
           <<"Wheel PI "<<(wheel_pi?"ON":"OFF (diagnostic; motor velocity mode retained)")<<'\n'
           <<"Wheel PI profile "<<profile_name<<"; crawl kp="<<actual_crawl_kp<<" ki="<<actual_crawl_ki<<"; normal kp="<<actual_normal_kp<<" ki="<<actual_normal_ki<<"; tau=0.12 s; normal cap=25%\n"
           <<"Integral gate "<<(pi_profile==WheelBalance::Profile::IntegralGate?"ON; target rate <=0.05 rad/s^2 for 0.40s; I gain transition rescaled":"OFF")<<'\n'
           <<"Control target 100 Hz; wheel feedback nominal 100 Hz; Bridge ready; "<<(hardware?"HARDWARE":"DRY RUN")<<"; waiting for neutral START command\n";
  std::string pending; Command c; bool have=false;
  using Clock=std::chrono::steady_clock;
  const auto period=std::chrono::milliseconds(10);
  auto deadline=Clock::now()+period;
  unsigned long missed_slots=0;
  while(running) {
   // Absolute schedule: no processing-time drift, no catch-up command bursts.
   auto current=Clock::now();
   if(current>deadline) {
    auto skipped=(current-deadline)/period+1;
    missed_slots+=static_cast<unsigned long>(skipped);
    deadline+=skipped*period;
   }
   std::this_thread::sleep_until(deadline);
   deadline+=period;
   char buf[4096]; ssize_t n;
   while((n=read(STDIN_FILENO,buf,sizeof(buf)))>0) {
    pending.append(buf,n); if(pending.size()>16384)throw std::runtime_error("command backlog overflow");
   }
   if(n==0) throw std::runtime_error("command pipe closed");
   if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)throw std::runtime_error("command read failed");
   size_t end;
   while((end=pending.find('\n'))!=std::string::npos) {
    auto fresh=parse(pending.substr(0,end));pending.erase(0,end+1);
    if(have&&fresh.stamp<c.stamp)throw std::runtime_error("non-monotonic command");
    c=fresh;have=true;
   }
   double t=now(),age=t-c.stamp,dt=t-last_loop;last_loop=t;
   if(have&&(age>.3||age<-.05))throw std::runtime_error("command watchdog expired (300 ms); restart required");
   if(!have)continue;
   if(c.enabled&&!powered) {
    if(std::abs(c.x)+std::abs(c.y)+std::abs(c.yaw)+std::abs(c.lift)>1e-8)
      throw std::runtime_error("first enable must be neutral");
    if(device){check(device->setChassisEnabled(true));if(lift)check(device->setLiftEnabled(true));}
    powered=true; std::cerr<<"Drives enabled at zero; awaiting next fresh command\n";
    // Enabling can block; discard all pre-enable commands, accept only later timestamps.
    double enabled_at=now(); c.enabled=0;c.x=c.y=c.yaw=c.lift=0;
    // Drain buffered old commands on next cycle by demanding a post-enable timestamp.
    while(running) {
     n=read(STDIN_FILENO,buf,sizeof(buf));
     if(n>0) {pending.append(buf,n);if(pending.size()>16384)throw std::runtime_error("enable backlog overflow");}
     else if(n==0)throw std::runtime_error("pipe closed during enable");
     while((end=pending.find('\n'))!=std::string::npos){auto f=parse(pending.substr(0,end));pending.erase(0,end+1);if(f.stamp>enabled_at)c=f;}
     if(c.stamp>enabled_at)break;
     if(now()-enabled_at>.3)throw std::runtime_error("no fresh command after enable");
     std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    continue;
   }
   poke::AuxiliaryState s;
   if(device&&powered){check(device->refreshChassisFeedback());s=device->state();}
   // Lift SDO is blocking: interlock chassis motion and lift motion for commissioning.
   bool chassis_request=c.enabled&&(std::abs(c.x)+std::abs(c.y)+std::abs(c.yaw)>1e-8);
   bool lift_request=c.enabled&&lift&&std::abs(c.lift)>.01&&!chassis_request;
   if(lift_moving&&!lift_request){if(device)check(device->stopLift());lift_moving=false;continue;}
   bool chassis_allowed=c.enabled&&!lift_moving&&!lift_request;
   auto measured_body=kin.forward(s.chassis_velocities);
   measured_body.vx_m_s*=-1;measured_body.vy_m_s*=-1;
   const bool body_active=device&&powered&&chassis_allowed&&chassis_request;
   poke::ChassisTwist body_target=chassis_allowed?poke::ChassisTwist{c.x,c.y,c.yaw}:poke::ChassisTwist{};
   if(body_active)body_target=body.update(body_target,measured_body,dt,s.chassis_velocity_feedback_valid,body_hold);
   else body.reset();
   // Final-front direction test 2026-09-17: legacy translation axes both reversed; +yaw verified CCW.
   auto sol=kin.inverse(poke::ChassisTwist{-body_target.vx_m_s,-body_target.vy_m_s,body_target.omega_z_rad_s});
   sol.applied_twist.vx_m_s *= -1; sol.applied_twist.vy_m_s *= -1; // Physical body axes.
   auto nominal=sol.wheel_rad_s;
   bool balance_active=wheel_pi&&device&&powered&&s.chassis_velocity_feedback_valid;
   if(balance_active)sol.wheel_rad_s=balance.update(nominal,s.chassis_velocities,dt,s.chassis_velocity_feedback_valid);
   else balance.reset();
   // Applied twist describes actual sent targets; requested remains the untrimmed user request.
   sol.applied_twist=kin.forward(sol.wheel_rad_s);
   sol.applied_twist.vx_m_s*=-1;sol.applied_twist.vy_m_s*=-1;
   if(device&&powered)check(device->setChassisWheelVelocities(sol.wheel_rad_s));
   // Read/jog lift only when chassis command and measured wheel velocities are zero.
   double peak=0;for(auto w:s.chassis_velocities)peak=std::max(peak,std::abs(w));
   if(lift&&powered&&!chassis_request&&peak<.1&&t-lift_read>=.2) {
    if(device){check(device->refreshLiftFeedback());s=device->state();}
    lift_read=now();
    if(lift_request&&now()-c.stamp<=.3&&running) {
     double target=std::clamp(s.lift_position_mm+c.lift*.6,0.0,250.0);
     if(device)check(device->setLiftPositionMm(target,3,false));
     lift_moving=true;
    }
   }
   if(device)s=device->state();
   if(log_every_cycle || t-last_log>=.05) {
    auto fb=kin.forward(s.chassis_velocities);
    fb.vx_m_s *= -1; fb.vy_m_s *= -1;
    out<<t<<','<<hardware<<','<<(powered&&c.enabled)<<','<<age<<','<<c.x<<','<<c.y<<','<<c.yaw<<','<<c.lift<<','<<sol.applied_twist.vx_m_s<<','<<sol.applied_twist.vy_m_s<<','<<sol.applied_twist.omega_z_rad_s<<','<<s.chassis_velocity_feedback_valid<<','<<s.chassis_velocity_feedback_max_age_ms;
    for(int i=0;i<4;++i)out<<','<<sol.wheel_rad_s[i]<<','<<s.chassis_velocities[i]<<','<<s.chassis_temperatures[i]<<','<<int(s.chassis_fault_codes[i]);
    out<<','<<fb.vx_m_s<<','<<fb.vy_m_s<<','<<fb.omega_z_rad_s<<','<<s.lift_enabled<<','<<s.lift_position_valid<<','<<(lift_read?now()-lift_read:-1)<<','<<s.lift_position_mm<<','<<s.lift_velocity_mm_s<<','<<s.lift_target_position_mm<<','<<s.lift_statusword<<','<<s.lift_error_code<<','<<s.lift_lower_limit_active<<','<<s.lift_upper_limit_active<<','<<dt*1000<<','<<missed_slots;
    for(int i=0;i<4;++i)out<<','<<nominal[i]<<','<<balance.correction[i]<<','<<balance.filtered[i];
    out<<','<<balance_active<<','<<body_active<<','<<body.holding<<','<<body.x<<','<<body.y<<','<<body.yaw<<','<<body.cross_error<<','<<body.cross_trim<<','<<body.yaw_trim<<','<<body_target.vx_m_s<<','<<body_target.vy_m_s<<','<<body_target.omega_z_rad_s<<','<<body.planar_correction<<','<<body.planar_limited;for(double v:balance.filtered_target)out<<','<<v;out<<','<<profile_name<<','<<actual_crawl_kp<<','<<actual_crawl_ki<<','<<actual_normal_kp<<','<<actual_normal_ki;
    for(int i=0;i<4;++i)out<<','<<balance.p_term[i]<<','<<balance.i_term[i]<<','<<balance.integral_enabled[i]<<','<<balance.stable_time[i]<<','<<balance.target_rate[i];
    out<<'\n';out.flush();
    if(!out)throw std::runtime_error("feedback log write failed");
    last_log=t;
   }
  }
 }catch(const std::exception&e){
  std::cerr<<"FAULT "<<e.what()<<'\n';result=1;
  if(device){auto s=device->state();std::cerr<<"FAULT SNAPSHOT wheel_valid="<<s.chassis_velocity_feedback_valid<<" age_ms="<<s.chassis_velocity_feedback_max_age_ms;
   for(int i=0;i<4;++i)std::cerr<<" motor"<<i+1<<"_fault="<<int(s.chassis_fault_codes[i]);
   std::cerr<<" lift_status="<<s.lift_statusword<<" lift_error_cached="<<s.lift_error_code<<'\n';}
 }
 cleanup(); std::cerr<<"Bridge exited; connected-device cleanup attempted (see any failures above)\n";return result;
}
