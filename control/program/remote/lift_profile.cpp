#include <ruckig/ruckig.hpp>
#include <cmath>
// Stateless trajectory evaluation from the preceding commanded velocity and
// acceleration; SI conversion is done in Python. No CAN or device writes.
extern "C" int lift_velocity_step(double v,double a,double target,double dt,
                                  double max_a,double max_j,double* result) {
  if (!result || !std::isfinite(v) || !std::isfinite(a) || !std::isfinite(target)
      || !std::isfinite(dt) || dt<=0 || dt>.3 || max_a<=0 || max_j<=0) return -1;
  try {
    ruckig::Ruckig<1> otg(dt);
    ruckig::InputParameter<1> input;
    ruckig::OutputParameter<1> output;
    input.control_interface=ruckig::ControlInterface::Velocity;
    input.current_position={0};input.current_velocity={v};input.current_acceleration={a};
    input.target_velocity={target};input.target_acceleration={0};
    input.max_acceleration={max_a};input.max_jerk={max_j};
    auto status=otg.update(input,output);
    if (status!=ruckig::Result::Working && status!=ruckig::Result::Finished) return int(status);
    result[0]=output.new_velocity[0];result[1]=output.new_acceleration[0];
    return 0;
  } catch (...) {return -2;}
}
