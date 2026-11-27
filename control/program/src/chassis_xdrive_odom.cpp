// Bundled "xdrive_odom" chassis-controller plugin.
//
// Ports the field-verified chassis path of the deployed poke-bot test console
// (backend.cpp, commissioned 2026-08-31): wheel-velocity odometry, the
// per-wheel bounded PI balance trim around the motors' internal velocity
// mode, and the command watchdog. The twist-shaping stage is new here: the
// console relied on its clients to ramp, this plugin shapes every desired
// twist through a Ruckig velocity-interface profile so the floating-base
// owner can hand it raw targets.

#include "poke/chassis_controller.hpp"

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>

namespace poke {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Wheel targets below this magnitude count as "stopped" for the balance trim
// and for direction-change detection (mirrors the deployed console).
constexpr double kMovingWheelThreshold = 1e-6;

// Substitute jerk cap used when jerk limiting is disabled: large enough that
// the acceleration limit alone bounds the profile at any supported loop rate
// (50-500 Hz: even a 2 ms step admits far more accel change than the caps).
constexpr double kEffectivelyUnlimitedJerk = 1e6;

// Feedback older than this reports unhealthy: about three RS02 type-24
// report periods (10 ms each) without a fresh sample. The criterion is
// wall-clock age, not tick counting, because at high control rates (e.g.
// 500 Hz) several ticks between 100 Hz feedback reports are routine — "no
// new sample this tick" is normal operation, only over-age feedback is a
// fault.
constexpr double kStaleFeedbackFaultMs = 30.0;

// Floor for the per-tick odometry integration cap. The cap is "two control
// periods" (as deployed at 50 Hz: 40 ms) so a caller hiccup cannot teleport
// the pose, but at 500 Hz two periods is only 4 ms while a lift-slot cycle
// legitimately blocks on SDO traffic for 10-50 ms; capping at 4 ms there
// silently drops most of that cycle's travel. The wheel sample is a cached
// measurement that stays valid for far longer than 40 ms, so integrating the
// real elapsed time up to that bound is the correct (and 50 Hz-identical)
// behaviour.
constexpr double kMinOdometryStepCapS = 0.040;

double wrapToPi(double angle) {
    angle = std::fmod(angle + kPi, 2.0 * kPi);
    if (angle < 0.0) angle += 2.0 * kPi;
    return angle - kPi;
}

// The balance integrator must not carry momentum across a stop or a reversal
// of its wheel's target; either would let the trim fight the new direction.
bool wheelTargetChangesDirection(double previous, double next) {
    return (std::abs(previous) > kMovingWheelThreshold &&
            std::abs(next) <= kMovingWheelThreshold) ||
           (std::abs(previous) <= kMovingWheelThreshold &&
            std::abs(next) > kMovingWheelThreshold) ||
           previous * next < 0.0;
}

class XDriveOdomController final : public IChassisController {
public:
    XDriveOdomController(const XDriveOdomConfig& config, ChassisWheelInterface io)
        : config_(config),
          io_(std::move(io)),
          kinematics_(config.wheel_radius_m, config.wheel_center_radius_m,
                      config.max_wheel_speed_rad_s),
          period_s_(1.0 / config.control_rate_hz),
          // Ruckig takes a fixed control period at construction; tick() must
          // therefore be called at config.control_rate_hz. dt_s only feeds
          // the odometry and balance integrals, which tolerate jitter.
          shaper_(period_s_) {
        shaper_input_.control_interface = ruckig::ControlInterface::Velocity;
        // The three planar DoFs must not stretch each other's ramps: each
        // axis reaches its own target as fast as its own limits allow.
        shaper_input_.synchronization = ruckig::Synchronization::None;
        double linear_jerk = config_.max_linear_jerk_m_s3;
        double angular_jerk = config_.max_angular_jerk_rad_s3;
        if (!std::isfinite(linear_jerk) || linear_jerk <= 0.0)
            linear_jerk = kEffectivelyUnlimitedJerk;
        if (!std::isfinite(angular_jerk) || angular_jerk <= 0.0)
            angular_jerk = kEffectivelyUnlimitedJerk;
        shaper_input_.max_velocity = {config_.max_linear_speed_m_s,
                                      config_.max_linear_speed_m_s,
                                      config_.max_angular_speed_rad_s};
        shaper_input_.max_acceleration = {config_.max_linear_acceleration_m_s2,
                                          config_.max_linear_acceleration_m_s2,
                                          config_.max_angular_acceleration_rad_s2};
        shaper_input_.max_jerk = {linear_jerk, linear_jerk, angular_jerk};
    }

    Status enable() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!io_.send_wheel_velocities || !io_.stop_wheels ||
            !io_.measured_wheel_velocities)
            return {ErrorCode::InvalidConfig,
                    "chassis wheel interface is incomplete"};
        pose_ = {};
        measured_twist_ = {};
        commanded_ = {};
        mode_ = Mode::Idle;
        has_target_ = false;
        target_ = {};
        twist_command_ = {};
        command_age_s_ = 0.0;
        feedback_age_ms_ = 0.0;
        io_fault_ = false;
        fault_reason_.clear();
        resetShaperLocked();
        resetBalanceLocked();
        previous_wheel_targets_.fill(0.0);
        // enable() must leave a defined stationary state before the first
        // tick has a chance to run.
        if (Status status = io_.stop_wheels(); !status) {
            fault_reason_ = "chassis stop failed on enable: " + status.message();
            return status;
        }
        enabled_ = true;
        healthy_ = true;
        return Status::ok();
    }

    Status disable() override {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = false;
        healthy_ = false;
        mode_ = Mode::Idle;
        has_target_ = false;
        commanded_ = {};
        resetShaperLocked();
        resetBalanceLocked();
        previous_wheel_targets_.fill(0.0);
        if (io_.stop_wheels) {
            if (Status status = io_.stop_wheels(); !status) {
                fault_reason_ =
                    "chassis stop failed on disable: " + status.message();
                return status;
            }
        }
        return Status::ok();
    }

    Status setTarget(const ChassisTarget& target) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
            return {ErrorCode::InvalidState, "chassis controller is not enabled"};
        if (!std::isfinite(target.x_m) || !std::isfinite(target.y_m) ||
            !std::isfinite(target.yaw_rad))
            return {ErrorCode::InvalidArgument, "non-finite chassis pose target"};
        target_ = target;
        has_target_ = true;
        mode_ = Mode::Target;
        command_age_s_ = 0.0;
        return Status::ok();
    }

    Status setTwist(const ChassisTwist& twist) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
            return {ErrorCode::InvalidState, "chassis controller is not enabled"};
        if (!std::isfinite(twist.vx_m_s) || !std::isfinite(twist.vy_m_s) ||
            !std::isfinite(twist.omega_z_rad_s))
            return {ErrorCode::InvalidArgument, "non-finite chassis body twist"};
        twist_command_ = twist;
        has_target_ = false;
        mode_ = Mode::Twist;
        command_age_s_ = 0.0;
        return Status::ok();
    }

    Status stop() override {
        std::lock_guard<std::mutex> lock(mutex_);
        // Dropping to Idle makes the next tick ramp to zero under the accel
        // and jerk limits; an instantaneous wheel stop is disable()'s job.
        mode_ = Mode::Idle;
        has_target_ = false;
        twist_command_ = {};
        command_age_s_ = 0.0;
        return Status::ok();
    }

    Status setCurrentAsOrigin() override {
        std::lock_guard<std::mutex> lock(mutex_);
        // The target is re-expressed in the new frame rather than dropped:
        // the physical goal (and therefore the tracking error and the
        // commanded twist) is continuous across the re-anchor, so an active
        // approach neither jumps nor aborts.
        if (has_target_) {
            const double dx = target_.x_m - pose_.x_m;
            const double dy = target_.y_m - pose_.y_m;
            const double cos_yaw = std::cos(pose_.yaw_rad);
            const double sin_yaw = std::sin(pose_.yaw_rad);
            target_.x_m = cos_yaw * dx + sin_yaw * dy;
            target_.y_m = -sin_yaw * dx + cos_yaw * dy;
            target_.yaw_rad = wrapToPi(target_.yaw_rad - pose_.yaw_rad);
        }
        pose_ = {};
        return Status::ok();
    }

    Status tick(double dt_s) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
            return {ErrorCode::InvalidState, "chassis controller is not enabled"};
        if (!std::isfinite(dt_s) || dt_s <= 0.0)
            return {ErrorCode::InvalidArgument,
                    "chassis tick dt must be finite and positive"};

        // 1) Feedback -> measured twist -> odometry.
        std::array<double, 4> measured{};
        double age_ms = 0.0;
        bool feedback_fresh = io_.measured_wheel_velocities(measured, age_ms);
        if (feedback_fresh)
            for (const double value : measured)
                if (!std::isfinite(value)) feedback_fresh = false;
        const bool was_over_age = feedback_age_ms_ > kStaleFeedbackFaultMs;
        if (feedback_fresh) {
            // The io layer serves the cached last sample with its wall-clock
            // age; the health criterion is that age, so ticks between two
            // 100 Hz feedback reports (routine at 500 Hz) are not faults.
            feedback_age_ms_ = std::max(age_ms, 0.0);
            measured_twist_ = kinematics_.forward(measured);
            // A timing hiccup in the caller must not teleport the odometry:
            // integration is capped at two control periods (as deployed at
            // 50 Hz), never below kMinOdometryStepCapS at higher rates.
            const double odometry_dt = std::min(
                dt_s, std::max(2.0 * period_s_, kMinOdometryStepCapS));
            if (!integrateMeasuredBodyTwist(pose_, measured_twist_,
                                            odometry_dt)) {
                fault_reason_ = "non-finite chassis planar odometry update";
                io_fault_ = true;
                healthy_ = false;
                mode_ = Mode::Idle;
                has_target_ = false;
                resetShaperLocked();
                resetBalanceLocked();
                (void)io_.stop_wheels();
                return {ErrorCode::InternalError, fault_reason_};
            }
        } else {
            // Commanded velocities are NOT a substitute for measurement:
            // without feedback the pose holds instead of dead-reckoning on
            // commands, and the balance trim loses its input. The io layer
            // reports no age for a rejected sample, so the last known age
            // keeps accumulating wall-clock time.
            feedback_age_ms_ += dt_s * 1000.0;
            resetBalanceLocked();
        }
        if (feedback_age_ms_ > kStaleFeedbackFaultMs && !was_over_age)
            fault_reason_ =
                "chassis wheel-velocity feedback stale: no fresh sample for " +
                std::to_string(static_cast<long>(std::lround(feedback_age_ms_))) +
                " ms (limit " +
                std::to_string(static_cast<long>(std::lround(kStaleFeedbackFaultMs))) +
                " ms)";

        // 2) Desired body twist for this tick.
        command_age_s_ += dt_s;
        if (mode_ != Mode::Idle &&
            command_age_s_ * 1000.0 > static_cast<double>(config_.command_watchdog_ms)) {
            mode_ = Mode::Idle;
            has_target_ = false;
            fault_reason_ =
                "chassis command watchdog expired; ramping wheel targets to zero";
        }
        ChassisTwist desired{};
        if (mode_ == Mode::Target) {
            const double error_x = target_.x_m - pose_.x_m;
            const double error_y = target_.y_m - pose_.y_m;
            const double error_yaw = wrapToPi(target_.yaw_rad - pose_.yaw_rad);
            const bool inside_tolerance =
                std::hypot(error_x, error_y) <= config_.translation_tolerance_m &&
                std::abs(error_yaw) <= config_.yaw_tolerance_rad;
            if (!inside_tolerance) {
                // World-frame P error rotated into the body frame the wheel
                // allocation expects.
                const double cos_yaw = std::cos(pose_.yaw_rad);
                const double sin_yaw = std::sin(pose_.yaw_rad);
                desired.vx_m_s = config_.translation_kp_s_inv *
                                 (cos_yaw * error_x + sin_yaw * error_y);
                desired.vy_m_s = config_.translation_kp_s_inv *
                                 (-sin_yaw * error_x + cos_yaw * error_y);
                desired.omega_z_rad_s = config_.yaw_kp_s_inv * error_yaw;
            }
        } else if (mode_ == Mode::Twist) {
            desired = twist_command_;
        }

        // 3) Caps. The linear cap scales the vector uniformly so the
        // commanded direction survives; clamping vx/vy independently would
        // skew diagonal motion.
        const double linear_speed = std::hypot(desired.vx_m_s, desired.vy_m_s);
        if (linear_speed > config_.max_linear_speed_m_s) {
            const double scale = config_.max_linear_speed_m_s / linear_speed;
            desired.vx_m_s *= scale;
            desired.vy_m_s *= scale;
        }
        desired.omega_z_rad_s = std::clamp(desired.omega_z_rad_s,
                                           -config_.max_angular_speed_rad_s,
                                           config_.max_angular_speed_rad_s);

        // 4) Accel/jerk shaping through the Ruckig velocity interface.
        shaper_input_.target_velocity = {desired.vx_m_s, desired.vy_m_s,
                                         desired.omega_z_rad_s};
        shaper_input_.target_acceleration = {0.0, 0.0, 0.0};
        const ruckig::Result shaping = shaper_.update(shaper_input_, shaper_output_);
        if (shaping != ruckig::Result::Working &&
            shaping != ruckig::Result::Finished) {
            fault_reason_ = "chassis twist shaping failed (ruckig result " +
                            std::to_string(static_cast<int>(shaping)) + ")";
            io_fault_ = true;
            healthy_ = false;
            resetShaperLocked();
            (void)io_.stop_wheels();
            return {ErrorCode::InternalError, fault_reason_};
        }
        shaper_output_.pass_to_input(shaper_input_);
        const ChassisTwist shaped{shaper_output_.new_velocity[0],
                                  shaper_output_.new_velocity[1],
                                  shaper_output_.new_velocity[2]};

        // 5) Wheel allocation (uniform scale-down inside the kinematics).
        // When the wheel cap engages, the shaper's internal state is left
        // alone on purpose: dragging it down to the scaled twist would keep
        // it accelerating both linear axes equally against the cap and skew
        // the commanded direction toward the diagonal. The shaper state is
        // already bounded by the body-twist caps, so it cannot run away.
        XDriveWheelSolution solution = kinematics_.inverse(shaped);
        commanded_ = solution.applied_twist;
        for (std::size_t index = 0; index < solution.wheel_rad_s.size(); ++index) {
            if (wheelTargetChangesDirection(previous_wheel_targets_[index],
                                            solution.wheel_rad_s[index])) {
                balance_integrals_[index] = 0.0;
                balance_corrections_[index] = 0.0;
            }
            previous_wheel_targets_[index] = solution.wheel_rad_s[index];
        }

        // 6) Per-wheel bounded PI trim around the motors' internal velocity
        // mode. The correction never exceeds balance_max_fraction of the
        // wheel target and never flips its sign; the integrator is clamped
        // so a persistently lagging wheel cannot wind it up.
        std::array<double, 4> send = solution.wheel_rad_s;
        if (config_.velocity_balance_enabled && feedback_fresh) {
            for (std::size_t index = 0; index < send.size(); ++index) {
                const double desired_wheel = solution.wheel_rad_s[index];
                if (std::abs(desired_wheel) < kMovingWheelThreshold) {
                    balance_integrals_[index] = 0.0;
                    balance_corrections_[index] = 0.0;
                    continue;
                }
                const double error = desired_wheel - measured[index];
                const double correction_limit =
                    config_.velocity_balance_max_fraction * std::abs(desired_wheel);
                if (config_.velocity_balance_ki > 0.0) {
                    const double integral_limit =
                        correction_limit / config_.velocity_balance_ki;
                    balance_integrals_[index] = std::clamp(
                        balance_integrals_[index] + error * dt_s,
                        -integral_limit, integral_limit);
                } else {
                    balance_integrals_[index] = 0.0;
                }
                const double correction = std::clamp(
                    config_.velocity_balance_kp * error +
                        config_.velocity_balance_ki * balance_integrals_[index],
                    -correction_limit, correction_limit);
                balance_corrections_[index] = correction;
                const double corrected = desired_wheel + correction;
                send[index] = desired_wheel > 0.0
                    ? std::clamp(corrected, 0.0, config_.max_wheel_speed_rad_s)
                    : std::clamp(corrected, -config_.max_wheel_speed_rad_s, 0.0);
            }
        }
        if (!feedback_fresh) {
            // Deployed semantics: without measurement the wheels are held at
            // zero rather than driven open-loop, and the shaper restarts from
            // standstill so recovery ramps instead of jumping.
            send.fill(0.0);
            commanded_ = {};
            resetShaperLocked();
        }

        // 7) Send. Every tick sends — including watchdog and stale-feedback
        // zeros — so a deadline stop actually reaches the wheels.
        if (Status send_status = io_.send_wheel_velocities(send); !send_status) {
            fault_reason_ =
                "chassis wheel send failed: " + send_status.message();
            io_fault_ = true;
            healthy_ = false;
            (void)io_.stop_wheels();
            return send_status;
        }
        io_fault_ = false;
        healthy_ = feedback_age_ms_ <= kStaleFeedbackFaultMs;
        return Status::ok();
    }

    ChassisControllerState state() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        ChassisControllerState snapshot;
        snapshot.enabled = enabled_;
        snapshot.healthy = healthy_;
        snapshot.pose = pose_;
        snapshot.twist = measured_twist_;
        snapshot.commanded = commanded_;
        snapshot.has_target = has_target_;
        snapshot.target = target_;
        snapshot.fault_reason = fault_reason_;
        return snapshot;
    }

private:
    enum class Mode { Idle, Target, Twist };

    void resetShaperLocked() {
        shaper_input_.current_velocity = {0.0, 0.0, 0.0};
        shaper_input_.current_acceleration = {0.0, 0.0, 0.0};
        shaper_input_.target_velocity = {0.0, 0.0, 0.0};
        shaper_input_.target_acceleration = {0.0, 0.0, 0.0};
    }

    void resetBalanceLocked() {
        balance_integrals_.fill(0.0);
        balance_corrections_.fill(0.0);
    }

    const XDriveOdomConfig config_;
    const ChassisWheelInterface io_;
    const XDriveKinematics kinematics_;
    const double period_s_;

    // Everything below is shared between the owner's tick() thread and
    // state() readers; one mutex covers it all (control-rate access,
    // contention-free).
    mutable std::mutex mutex_;
    ruckig::Ruckig<3> shaper_;
    ruckig::InputParameter<3> shaper_input_;
    ruckig::OutputParameter<3> shaper_output_;
    bool enabled_{false};
    bool healthy_{false};
    bool io_fault_{false};
    // Wall-clock age of the newest usable wheel-feedback sample, advanced by
    // dt on ticks where the io layer rejects the sample.
    double feedback_age_ms_{0.0};
    Mode mode_{Mode::Idle};
    bool has_target_{false};
    ChassisTarget target_{};
    ChassisTwist twist_command_{};
    double command_age_s_{0.0};
    ChassisPose2d pose_{};
    ChassisTwist measured_twist_{};
    ChassisTwist commanded_{};
    std::array<double, 4> previous_wheel_targets_{};
    std::array<double, 4> balance_integrals_{};
    std::array<double, 4> balance_corrections_{};
    std::string fault_reason_;
};

bool validXDriveOdomConfig(const XDriveOdomConfig& config) {
    const auto positive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    const auto non_negative = [](double value) {
        return std::isfinite(value) && value >= 0.0;
    };
    return positive(config.wheel_radius_m) &&
           positive(config.wheel_center_radius_m) &&
           positive(config.max_wheel_speed_rad_s) &&
           positive(config.max_linear_speed_m_s) &&
           positive(config.max_angular_speed_rad_s) &&
           positive(config.max_linear_acceleration_m_s2) &&
           positive(config.max_angular_acceleration_rad_s2) &&
           non_negative(config.translation_kp_s_inv) &&
           non_negative(config.yaw_kp_s_inv) &&
           non_negative(config.translation_tolerance_m) &&
           non_negative(config.yaw_tolerance_rad) &&
           non_negative(config.velocity_balance_kp) &&
           non_negative(config.velocity_balance_ki) &&
           std::isfinite(config.velocity_balance_max_fraction) &&
           config.velocity_balance_max_fraction > 0.0 &&
           config.velocity_balance_max_fraction <= 1.0 &&
           config.command_watchdog_ms > 0 &&
           positive(config.control_rate_hz);
}

}  // namespace

std::unique_ptr<IChassisController> makeChassisController(
    const std::string& type,
    const XDriveOdomConfig& config,
    ChassisWheelInterface io) {
    if (type != "xdrive_odom") return nullptr;
    // The factory has no status channel, so an invalid configuration degrades
    // to nullptr exactly like an unknown type instead of throwing from the
    // kinematics constructor.
    if (!validXDriveOdomConfig(config)) return nullptr;
    return std::make_unique<XDriveOdomController>(config, std::move(io));
}

}  // namespace poke
