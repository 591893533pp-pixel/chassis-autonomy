#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
// Bounded outer PI, same nominal gains as XDriveOdomConfig. Not a localization loop.
struct WheelBalance {
 enum class Profile { Baseline, SmoothCandidate, IntegralGate };
 static constexpr double integral_gate_rate_rad_s2=.05;
 static constexpr double integral_gate_settle_s=.40;
 const double max_speed;
 const Profile profile;
 const double custom_crawl_kp, custom_crawl_ki, custom_normal_kp, custom_normal_ki;
 explicit WheelBalance(double maximum=1.0,Profile selected=Profile::Baseline,
                       double crawl_kp=std::numeric_limits<double>::quiet_NaN(),
                       double crawl_ki=std::numeric_limits<double>::quiet_NaN(),
                       double normal_kp=std::numeric_limits<double>::quiet_NaN(),
                       double normal_ki=std::numeric_limits<double>::quiet_NaN()):
  max_speed(maximum),profile(selected),custom_crawl_kp(crawl_kp),custom_crawl_ki(crawl_ki),
  custom_normal_kp(normal_kp),custom_normal_ki(normal_ki){
  if(!std::isfinite(maximum)||maximum<=0)throw std::runtime_error("invalid wheel speed limit");
  for(double v:{crawl_kp,crawl_ki,normal_kp,normal_ki})
   if(std::isfinite(v)&&v<0)throw std::runtime_error("invalid wheel PI gain");
 }
 std::array<double,4> integral{}, filtered{}, filtered_target{}, previous{}, correction{};
 std::array<double,4> previous_ki{}, stable_time{}, target_rate{}, p_term{}, i_term{};
 std::array<bool,4> integral_enabled{};
 bool initialized=false;
 void reset(){integral={};filtered={};filtered_target={};previous={};correction={};
  previous_ki={};stable_time={};target_rate={};p_term={};i_term={};integral_enabled={};initialized=false;}
 std::array<double,4> update(const std::array<double,4>& target,
                            const std::array<double,4>& measured,double dt,bool fresh){
  if(!fresh){reset();throw std::runtime_error("wheel balance requires fresh feedback");}
  for(int i=0;i<4;++i)if(!std::isfinite(target[i])||!std::isfinite(measured[i])||std::abs(target[i])>max_speed+1e-6){reset();throw std::runtime_error("invalid wheel balance input");}
  if(!std::isfinite(dt)||dt<=0||dt>.1){reset();return {};}
  // At crawl speed the encoder quantization and CAN sample jitter are a
  // sizeable fraction of the requested wheel speed.  Filter longer and use
  // a deliberately softer PI loop in that region so the correction cannot
  // turn measurement noise into an S-shaped trajectory.
  const double alpha=dt/(.12+dt);
  if(!initialized){filtered=measured;filtered_target=target;initialized=true;}
  std::array<double,4> result{};
  for(int i=0;i<4;++i){
   filtered[i]+=alpha*(measured[i]-filtered[i]);
   filtered_target[i]+=alpha*(target[i]-filtered_target[i]);
   if(std::abs(target[i])<1e-6){
    integral[i]=correction[i]=previous[i]=filtered_target[i]=0;
    previous_ki[i]=stable_time[i]=target_rate[i]=p_term[i]=i_term[i]=0;
    integral_enabled[i]=false;continue;
   }
   if(target[i]*previous[i]<=0){
    integral[i]=0;filtered[i]=measured[i];filtered_target[i]=target[i];
    previous_ki[i]=stable_time[i]=0;
   }
   const bool crawl=std::abs(target[i]<0? -target[i] : target[i]) < .80;
   // Opt-in candidate screened offline on 0.2--0.5 m/s data. Actual motor
   // smoothing is not yet verified. Keep baseline and crawl behavior intact.
   const bool soft=crawl || profile==Profile::SmoothCandidate;
   const double kp=crawl ? (std::isfinite(custom_crawl_kp)?custom_crawl_kp:(soft?.12:.20))
                          : (std::isfinite(custom_normal_kp)?custom_normal_kp:(soft?.12:.20));
   const double ki=crawl ? (std::isfinite(custom_crawl_ki)?custom_crawl_ki:(soft?.12:.30))
                          : (std::isfinite(custom_normal_ki)?custom_normal_ki:(soft?.12:.30));
   const double limit_fraction=crawl?.15:.25;
   // Match reference and measurement bandwidths. Filtering only measurement
   // creates an artificial tracking error on a ramp, even with perfect motors.
   // Keep the unfiltered target as feedforward; stop still bypasses the filter.
   const double error=filtered_target[i]-filtered[i],limit=limit_fraction*std::abs(target[i]);
   const bool gated=profile==Profile::IntegralGate;
   target_rate[i]=(target[i]-previous[i])/dt;
   if(gated){
    // Freeze accumulation on ramps. Nominal target determines the gate;
    // noisy feedback and PI-adjusted commands never drive the derivative.
    if(std::abs(target_rate[i])<=integral_gate_rate_rad_s2)
     stable_time[i]=std::min(integral_gate_settle_s,stable_time[i]+dt);
    else stable_time[i]=0;
    // Preserve I output through gain changes; existing safety caps still win
    // if a smaller target makes the previous output infeasible. P is unchanged.
    if(previous_ki[i]>0 && previous_ki[i]!=ki)integral[i]*=previous_ki[i]/ki;
    integral[i]=std::clamp(integral[i],-limit/ki,limit/ki);
   }
   integral_enabled[i]=!gated || stable_time[i]>=integral_gate_settle_s-1e-12;
   const double trial=std::clamp(integral[i]+(integral_enabled[i]?error*dt:0.),-limit/ki,limit/ki);
   const double unsaturated=kp*error+ki*trial;
   // Conditional integration: no accumulating error while pushing into saturation.
   if(integral_enabled[i] && (std::abs(unsaturated)<=limit || unsaturated*error<0))integral[i]=trial;
   p_term[i]=kp*error;i_term[i]=ki*integral[i];previous_ki[i]=ki;
   const double trim=std::clamp(p_term[i]+i_term[i],-limit,limit);
   result[i]=target[i]>0?std::clamp(target[i]+trim,0.,max_speed):std::clamp(target[i]+trim,-max_speed,0.);
   correction[i]=result[i]-target[i];previous[i]=target[i];
  }
  return result;
 }
};
