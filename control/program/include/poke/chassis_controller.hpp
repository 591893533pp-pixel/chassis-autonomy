#pragma once

// Chassis motion-controller plugin boundary.
//
// The floating-base layer (poke/base.hpp) owns the planar x/y/yaw degrees of
// freedom through this interface only. The bundled "xdrive_odom"
// implementation closes the loop on wheel-velocity odometry and drives the
// four X-drive omni wheels; a future SLAM-localized external controller
// replaces the whole implementation behind this interface without touching
// the floating-base API or the lift.
//
// Frame convention: targets and state poses live in the controller's world
// frame — for xdrive_odom that is the odometry frame established at enable
// (or the latest setCurrentAsOrigin()). +x forward, +y left, yaw
// counter-clockwise about +z, matching poke/chassis_kinematics.hpp.

#include "poke/chassis_kinematics.hpp"
#include "poke/chassis_odometry.hpp"
#include "poke/status.hpp"

#include <array>
#include <functional>
#include <memory>
#include <string>

namespace poke {

struct ChassisTarget {
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
};

struct ChassisControllerState {
    bool enabled{false};
    bool healthy{false};
    ChassisPose2d pose{};        // world/odometry frame
    ChassisTwist twist{};        // measured body twist
    ChassisTwist commanded{};    // last commanded body twist (post-shaping)
    bool has_target{false};
    ChassisTarget target{};
    std::string fault_reason;
};

// Hardware access handed to a chassis-controller implementation by the
// floating-base owner. Implementations never talk to CAN directly; this keeps
// them unit-testable and keeps bus ownership in one place.
struct ChassisWheelInterface {
    // Command the four wheel velocities (rad/s, motor order/sign convention of
    // XDriveKinematics).
    std::function<Status(const std::array<double, 4>&)> send_wheel_velocities;
    // Immediately command zero on all wheels.
    std::function<Status()> stop_wheels;
    // Latest measured wheel velocities. Returns false when feedback is
    // missing or older than the hardware-layer freshness bound.
    std::function<bool(std::array<double, 4>& velocities_rad_s,
                       double& age_ms)> measured_wheel_velocities;
};

// Configuration for the bundled xdrive_odom implementation. Values mirror the
// deployed poke-bot chassis commissioning (backend.yaml, verified 2026-08-31).
struct XDriveOdomConfig {
    // Geometry.
    double wheel_radius_m{0.0762};
    double wheel_center_radius_m{0.25213};
    double max_wheel_speed_rad_s{9.424777960769};  // 90 rpm supplier ceiling

    // Body-twist caps applied before wheel allocation.
    double max_linear_speed_m_s{1.015643039663};
    double max_angular_speed_rad_s{0.20};
    double max_linear_acceleration_m_s2{0.15};
    double max_angular_acceleration_rad_s2{0.40};
    // Jerk limits for the Ruckig velocity-interface shaping stage. Zero or
    // non-finite disables jerk limiting (acceleration-limited profile only).
    double max_linear_jerk_m_s3{1.0};
    double max_angular_jerk_rad_s3{2.0};

    // Pose-tracking P gains (target mode). Velocity feed-forward is not used
    // by the position loop yet; the teleop planner historically ran kp = 1/s.
    double translation_kp_s_inv{1.0};
    double yaw_kp_s_inv{1.0};
    double translation_tolerance_m{0.005};
    double yaw_tolerance_rad{0.01};

    // Per-wheel bounded PI trim around the motors' internal velocity mode,
    // driven by RS02 type-24 feedback. The trim may change a wheel target by
    // at most balance_max_fraction and never changes its sign.
    bool velocity_balance_enabled{true};
    double velocity_balance_kp{0.20};
    double velocity_balance_ki{0.30};
    double velocity_balance_max_fraction{0.25};

    // No fresh target/twist within this window -> ramp to zero and hold.
    uint32_t command_watchdog_ms{750};

    // Control loop rate the owner promises to call tick() at (the Ruckig
    // twist-shaping period is derived from it). The floating-base owner
    // overwrites this with BaseConfig::control_rate_hz, since its control
    // thread is the tick() driver; the default only applies to standalone
    // plugin use.
    double control_rate_hz{50.0};
};

class IChassisController {
public:
    virtual ~IChassisController() = default;

    // Lifecycle. enable() must leave the controller stationary with a defined
    // world frame; disable() must stop the wheels.
    virtual Status enable() = 0;
    virtual Status disable() = 0;

    // World-frame pose target (position-tracking mode). Repeated calls
    // update the target; the controller closes the loop internally.
    virtual Status setTarget(const ChassisTarget& target) = 0;

    // Direct body-twist bypass (velocity mode). Mutually exclusive with
    // setTarget: the most recent call wins.
    virtual Status setTwist(const ChassisTwist& twist) = 0;

    // Drop any target and ramp to zero.
    virtual Status stop() = 0;

    // Re-anchor the world frame at the current pose (x = y = yaw = 0).
    virtual Status setCurrentAsOrigin() = 0;

    // One control step: consume feedback, update odometry/estimation, run the
    // tracking loop, and command the wheels. Called by the floating-base
    // control thread at the configured rate; dt_s is the elapsed time since
    // the previous tick.
    virtual Status tick(double dt_s) = 0;

    virtual ChassisControllerState state() const = 0;
};

// Factory. `type` currently supports "xdrive_odom".
std::unique_ptr<IChassisController> makeChassisController(
    const std::string& type,
    const XDriveOdomConfig& config,
    ChassisWheelInterface io);

}  // namespace poke
