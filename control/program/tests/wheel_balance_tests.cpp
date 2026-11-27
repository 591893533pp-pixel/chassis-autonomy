#include "wheel_balance.hpp"
#include <stdexcept>
static void check(bool b){if(!b)throw std::runtime_error("balance test failed");}
int main(){
 WheelBalance b;std::array<double,4> goal{.2,-.2,.2,-.2},measured{};
 auto out=b.update(goal,measured,.02,true);
 for(int i=0;i<4;++i){check(out[i]*goal[i]>0);check(std::abs(out[i])>std::abs(goal[i]));check(std::abs(out[i]-goal[i])<=.05+1e-9);}
 // Simulated first-order motor with 15% steady speed loss: bounded outer loop reduces error.
 b.reset();measured={};
 for(int k=0;k<1000;++k){out=b.update(goal,measured,.02,true);for(int i=0;i<4;++i)measured[i]+=.2*(.85*out[i]-measured[i]);}
 for(int i=0;i<4;++i)check(std::abs(measured[i]-goal[i])<.005);
 out=b.update({},measured,.02,true);for(auto x:out)check(x==0);for(auto x:b.integral)check(x==0);
 measured.fill(10);for(int k=0;k<100;++k){out=b.update(goal,measured,.02,true);for(int i=0;i<4;++i){check(out[i]*goal[i]>=0);check(std::abs(out[i]-goal[i])<=.05+1e-9);}}
 bool threw=false;try{b.update(goal,measured,.02,false);}catch(...){threw=true;}check(threw);
 out=b.update(goal,{},.3,true);for(auto x:out)check(x==0);
 // Perfect tracking during ramp-up and ramp-down must not create trim from
 // measurement-filter delay. Test all four sign patterns and a reversal.
 b.reset();
 for(double sign:{1.,-1.}){
  for(int k=0;k<150;++k){
   double v=.464*std::min((k+1)*.02/.5,1.);
   if(k>100)v=.464*(149-k)/49.;
   std::array<double,4> ramp{sign*v,-sign*v,-sign*v,sign*v};
   out=b.update(ramp,ramp,.02,true);
   for(int i=0;i<4;++i){check(std::abs(out[i]-ramp[i])<1e-12);check(std::abs(b.integral[i])<1e-12);}
  }
 }
 // Expanded remote limit must handle both directions, saturate and stop.
 WheelBalance fast(10.0);
 std::array<double,4> high{9.8,-9.8,6.5,-6.5};
 for(int k=0;k<100;++k){out=fast.update(high,{},.02,true);for(int i=0;i<4;++i){check(out[i]*high[i]>=0);check(std::abs(out[i])<=10.);}}
 out=fast.update({},high,.02,true);for(double v:out)check(v==0);
 threw=false;try{fast.update({10.1,0,0,0},{},.02,true);}catch(...){threw=true;}check(threw);
 // Candidate gains act on the command, not on plotted feedback. Known initial
 // 1 rad/s error gives P*error + I*error*dt, in both wheel directions.
 WheelBalance soft(10.,WheelBalance::Profile::SmoothCandidate);
 std::array<double,4> ref{2,-2,-2,2},fb{1,-1,-1,1};
 out=soft.update(ref,fb,.02,true);
 for(int i=0;i<4;++i)check(std::abs(out[i]-ref[i]-(ref[i]>0?1:-1)*.1224)<1e-12);
 // Identical crawl behavior; high-speed target crossings introduce no gain
 // discontinuity for the candidate, and perfect tracking adds no correction.
 WheelBalance legacy(10.);
 soft.reset();
 for(int k=1;k<80;++k){
  double v=.009*k;ref={v,-v,-v,v};fb={.9*v,-.9*v,-.9*v,.9*v};
  auto a=legacy.update(ref,fb,.02,true),b=soft.update(ref,fb,.02,true);
  for(int i=0;i<4;++i)check(a[i]==b[i]);
 }
 soft.reset();
 for(int k=0;k<200;++k){
  double v=5.*std::min((k+1)*.02/2.5,1.);ref={v,-v,-v,v};
  out=soft.update(ref,ref,.02,true);
  for(int i=0;i<4;++i)check(std::abs(out[i]-ref[i])<1e-12);
 }
 // Candidate retains saturation, stop, stale-feedback and bad-timestep guards.
 ref={9.8,-9.8,-9.8,9.8};fb={-40,40,40,-40};
 for(int k=0;k<200;++k){out=soft.update(ref,fb,.02,true);for(int i=0;i<4;++i){check(out[i]*ref[i]>=0);check(std::abs(out[i])<=10.);}}
 out=soft.update({},fb,.02,true);for(auto v:out)check(v==0);for(auto v:soft.integral)check(v==0);
 threw=false;try{soft.update(ref,fb,.02,false);}catch(...){threw=true;}check(threw);
 out=soft.update(ref,fb,.2,true);for(auto v:out)check(v==0);
}
