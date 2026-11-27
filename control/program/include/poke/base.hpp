#pragma once

// Floating-base facade: one 4-DoF interface (x, y, z, yaw) over the mobile
// chassis (x/y/yaw via an IChassisController plugin) and the lift column
// (z via the auxiliary ZeroErr CANopen Profile Position drive).
//
// Upstream (teleop, planners) commands a floating-base pose and never sees
// wheels, screws, counts, or CAN. Frames: x/y/yaw live in the chassis
// controller's world frame (odometry frame for xdrive_odom); z is the
// absolute lift height in metres above the lift's calibrated zero
// (0 .. lift_max_position_mm/1000).

#include "poke/chassis_controller.hpp"
#include "poke/status.hpp"
#include "poke/types.hpp"

#include <memory>
#include <string>

namespace poke {

namespace detail { class AuxiliaryController; }

struct BaseTarget {
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    double yaw_rad{0.0};
};

struct BaseState {
    // Pose (world/odometry frame; z absolute lift height in metres).
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    double yaw_rad{0.0};
    // Twist.
    double vx_m_s{0.0};
    double vy_m_s{0.0};
    double vz_m_s{0.0};
    double omega_z_rad_s{0.0};
    // Health.
    bool chassis_enabled{false};
    bool chassis_healthy{false};
    bool lift_enabled{false};
    bool lift_healthy{false};
    bool has_target{false};
    BaseTarget target{};
    std::string fault_reason;
};

struct BaseConfig {
    // Control thread rate. The chassis is ticked every cycle (the plugin's
    // own control_rate_hz is overridden with this value, so the Ruckig
    // shaping period always matches the real tick rate); lift Profile
    // Position setpoints are decimated to lift_command_rate_hz via a ceiling
    // divider, so the lift cadence can be configured independently of the
    // control rate.
    //
    // WARNING (lift_command_rate_hz = control_rate_hz = 500): the CiA-402
    // SDO handshake is capped at ~20 Hz on the deployed adapter, and lift
    // feedback/setpoints are *blocking* SDO transactions executed inside the
    // control thread. At 500 Hz the ceiling divider becomes 1, so every 2 ms
    // cycle performs those blocking reads (plus a setpoint write while
    // moving), which overruns the cycle budget by one to two orders of
    // magnitude and drags the chassis tick rate down to the SDO turnaround
    // rate. Until lift IO moves to its own thread, deployments driving both
    // DoFs on the current adapter should configure lift_command_rate_hz back
    // down (<= 20 Hz); the decimation mechanism is retained for exactly that.
    double control_rate_hz{500.0};
    double lift_command_rate_hz{500.0};

    // Chassis plugin selection + configuration.
    std::string chassis_controller{"xdrive_odom"};
    XDriveOdomConfig chassis{};

    // Lift target shaping (Ruckig<1> on z). Velocity/acceleration in m/s and
    // m/s^2 at the carriage; the profile velocity handed to the drive is
    // derived from the shaped trajectory and clamped to the AuxiliaryConfig
    // lift_max_profile_velocity_mm_s.
    double lift_max_speed_m_s{0.02};
    double lift_max_acceleration_m_s2{0.05};
    double lift_max_jerk_m_s3{0.20};
};

class BaseController {
public:
    // The auxiliary configuration must have enable_chassis/enable_lift and
    // the metric lift parameters populated (LiftControlMode::
    // MetricProfilePosition) for the corresponding DoF to be served.
    // Construction cannot return Status: configuration errors (including an
    // unknown chassis_controller type) are latched and surfaced by
    // connect()/enable().
    BaseController(AuxiliaryConfig auxiliary, BaseConfig base);
    ~BaseController();

    BaseController(const BaseController&) = delete;
    BaseController& operator=(const BaseController&) = delete;

    // Lifecycle: connect() opens the auxiliary CAN adapter; enable() powers
    // chassis + lift and starts the control thread stationary; disable()
    // stops motion, powers drives down, and joins the thread.
    Status connect();
    Status enable();
    Status disable();
    void close();

    // Command the floating base. Chassis DoFs track the pose target through
    // the plugin; z is shaped and streamed as Profile Position waypoints.
    Status setTarget(const BaseTarget& target);

    // Immediately drop targets, ramp the chassis to zero, halt the lift.
    Status stop();

    // Re-anchor x = y = yaw = 0 at the current pose. z is absolute and
    // unaffected.
    Status setCurrentAsOrigin();

    BaseState state() const;

    // Escape hatch to the internally-owned CAN hardware layer, for
    // commissioning/jog/diagnostic paths only (the auxiliary controller is
    // the single owner of the adapter, so low-level access must go through
    // this instance). Callers must not fight the control thread for the same
    // DoF: pause targets with stop(), or keep the base disabled, while
    // jogging.
    detail::AuxiliaryController& auxiliary();
    const detail::AuxiliaryController& auxiliary() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace poke
