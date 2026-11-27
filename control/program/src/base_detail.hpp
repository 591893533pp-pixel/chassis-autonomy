#pragma once

// Per-cycle decision logic for the floating-base control thread, factored out
// of BaseController so it is unit-testable without CAN hardware. The thread in
// src/base.cpp is a thin driver: it refreshes feedback, asks BaseCycleLogic
// what to do this cycle, and forwards the resulting calls to the chassis
// plugin and the auxiliary lift.

#include "poke/base.hpp"
#include "poke/chassis_controller.hpp"
#include "poke/status.hpp"
#include "poke/types.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

namespace poke::detail {

// The drive rejects a zero profile velocity; segments that end at rest still
// need a positive crawl speed to reach the final waypoint.
inline constexpr double kLiftProfileVelocityFloorMmS = 1.0;

// Shaped waypoints closer than this are "the same target" for resend
// suppression; the SDO channel is only re-touched when the target moves.
inline constexpr double kLiftResendEpsilonMm = 1e-6;

// Lift feedback consumed by the cycle logic (subset of AuxiliaryState so
// tests can fabricate it without hardware).
struct BaseLiftFeedback {
    bool position_valid{false};
    double position_mm{0.0};
    double velocity_mm_s{0.0};
    bool target_reached{false};
};

struct BaseLiftCommand {
    bool send{false};
    double position_mm{0.0};
    double profile_velocity_mm_s{0.0};
};

// What the control thread must do this cycle. Chassis target pushes happen
// before the tick so the plugin tracks the newest pose within the same cycle.
struct BaseCycleDecision {
    bool tick_chassis{false};
    bool push_chassis_target{false};
    ChassisTarget chassis_target{};
    bool lift_slot{false};
    BaseLiftCommand lift{};
};

// Joint validation of the auxiliary/base configuration pair. Errors here are
// what BaseController::connect()/enable() surface for a bad config.
inline Status validateBaseConfig(const AuxiliaryConfig& aux,
                                 const BaseConfig& base) {
    if (!aux.enable_chassis && !aux.enable_lift)
        return {ErrorCode::InvalidConfig,
                "floating base requires enable_chassis and/or enable_lift"};
    if (!std::isfinite(base.control_rate_hz) || base.control_rate_hz <= 0.0 ||
        base.control_rate_hz > 1000.0)
        return {ErrorCode::InvalidConfig,
                "base control_rate_hz must be finite in (0, 1000]"};
    if (aux.enable_lift) {
        if (!std::isfinite(base.lift_command_rate_hz) ||
            base.lift_command_rate_hz <= 0.0)
            return {ErrorCode::InvalidConfig,
                    "lift_command_rate_hz must be finite and positive"};
        if (base.lift_command_rate_hz > base.control_rate_hz + 1e-9)
            return {ErrorCode::InvalidConfig,
                    "lift_command_rate_hz cannot exceed control_rate_hz"};
        if (aux.lift_control_mode != LiftControlMode::MetricProfilePosition)
            return {ErrorCode::InvalidConfig,
                    "floating base requires LiftControlMode::"
                    "MetricProfilePosition on the lift"};
        if (!(aux.liftCountsPerMm() > 0.0))
            return {ErrorCode::InvalidConfig,
                    "lift metric parameters (counts/rev, reduction, screw "
                    "lead) must be populated"};
        if (!(aux.lift_max_position_mm > aux.lift_min_position_mm))
            return {ErrorCode::InvalidConfig,
                    "lift_max_position_mm must exceed lift_min_position_mm"};
        if (!std::isfinite(aux.lift_max_profile_velocity_mm_s) ||
            aux.lift_max_profile_velocity_mm_s <= 0.0)
            return {ErrorCode::InvalidConfig,
                    "lift_max_profile_velocity_mm_s must be positive"};
        if (!std::isfinite(base.lift_max_speed_m_s) ||
            base.lift_max_speed_m_s <= 0.0 ||
            !std::isfinite(base.lift_max_acceleration_m_s2) ||
            base.lift_max_acceleration_m_s2 <= 0.0 ||
            !std::isfinite(base.lift_max_jerk_m_s3) ||
            base.lift_max_jerk_m_s3 <= 0.0)
            return {ErrorCode::InvalidConfig,
                    "lift shaping limits (speed/accel/jerk) must be positive"};
    }
    if (aux.enable_chassis && base.chassis_controller.empty())
        return {ErrorCode::InvalidConfig, "chassis_controller type is empty"};
    return Status::ok();
}

// Jerk-limited single-axis position shaping (ruckig::Ruckig<1> behind a pimpl
// so this header stays includable from the test target, which links poke_sdk
// but not ruckig directly). All units are drive units: mm, mm/s, mm/s^2.
class LiftShaper {
public:
    LiftShaper(double period_s, double max_velocity_mm_s,
               double max_acceleration_mm_s2, double max_jerk_mm_s3);
    ~LiftShaper();

    LiftShaper(const LiftShaper&) = delete;
    LiftShaper& operator=(const LiftShaper&) = delete;

    // Restart shaping from a measured position at rest.
    void prime(double position_mm);
    void setTargetMm(double target_mm);

    struct Step {
        bool ok{false};
        double position_mm{0.0};
        // Shaped velocity at both ends of the segment; the profile velocity
        // handed to the drive covers the faster end.
        double velocity_before_mm_s{0.0};
        double velocity_after_mm_s{0.0};
        int ruckig_result{0};
    };
    Step step();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class BaseCycleLogic {
public:
    BaseCycleLogic(const AuxiliaryConfig& aux, const BaseConfig& base)
        : chassis_served_(aux.enable_chassis),
          lift_served_(aux.enable_lift),
          lift_min_mm_(aux.lift_min_position_mm),
          lift_max_mm_(aux.lift_max_position_mm),
          lift_profile_ceiling_mm_s_(aux.lift_max_profile_velocity_mm_s),
          lift_divider_(liftDividerFor(base)),
          lift_period_s_(lift_divider_ / base.control_rate_hz) {
        // Shaping runs in drive units (mm). The shaped speed must respect the
        // drive's profile-velocity ceiling or waypoints outrun the carriage.
        if (lift_served_)
            shaper_ = std::make_unique<LiftShaper>(
                lift_period_s_,
                std::min(base.lift_max_speed_m_s * 1000.0,
                         lift_profile_ceiling_mm_s_),
                base.lift_max_acceleration_m_s2 * 1000.0,
                base.lift_max_jerk_m_s3 * 1000.0);
    }

    int liftDivider() const { return lift_divider_; }
    double liftPeriodS() const { return lift_period_s_; }
    bool chassisServed() const { return chassis_served_; }
    bool liftServed() const { return lift_served_; }

    double clampZTargetM(double z_m) const {
        return std::clamp(z_m * 1000.0, lift_min_mm_, lift_max_mm_) / 1000.0;
    }

    void setTarget(const BaseTarget& target) {
        target_ = target;
        target_.z_m = clampZTargetM(target.z_m);
        has_target_ = true;
        chassis_target_dirty_ = chassis_served_;
    }

    void clearTarget() {
        // The lift halts at an unknown position; re-prime shaping from
        // measured feedback when the next target arrives.
        has_target_ = false;
        chassis_target_dirty_ = false;
        lift_primed_ = false;
        last_sent_position_mm_.reset();
    }

    bool hasTarget() const { return has_target_; }
    const BaseTarget& target() const { return target_; }
    const std::string& shapingFault() const { return shaping_fault_; }

    // True when the upcoming onCycle() call lands on a lift decimation slot;
    // the driver refreshes lift feedback only for those cycles (the refresh is
    // a blocking SDO read). With lift_command_rate_hz == control_rate_hz the
    // divider is 1 and *every* cycle blocks on lift SDO traffic — on the
    // deployed ~20 Hz-handshake adapter that overruns a 2 ms (500 Hz) budget
    // by orders of magnitude; configure lift_command_rate_hz down (<= 20 Hz)
    // until lift IO runs on its own thread (see BaseConfig).
    bool liftDue() const {
        return lift_served_ && (cycle_index_ % lift_divider_ == 0);
    }

    BaseCycleDecision onCycle(const BaseLiftFeedback& lift_feedback) {
        BaseCycleDecision decision;
        decision.tick_chassis = chassis_served_;
        if (chassis_served_ && has_target_ && chassis_target_dirty_) {
            decision.push_chassis_target = true;
            decision.chassis_target = {target_.x_m, target_.y_m,
                                       target_.yaw_rad};
            chassis_target_dirty_ = false;
        }
        decision.lift_slot = liftDue();
        if (decision.lift_slot) decision.lift = liftStep(lift_feedback);
        ++cycle_index_;
        return decision;
    }

private:
    // Ceiling divider: the realized lift cadence never exceeds
    // lift_command_rate_hz even when the rates do not divide evenly.
    static int liftDividerFor(const BaseConfig& base) {
        return std::max(
            1, static_cast<int>(std::ceil(
                   base.control_rate_hz /
                   std::max(base.lift_command_rate_hz, 1e-9))));
    }

    BaseLiftCommand liftStep(const BaseLiftFeedback& feedback) {
        BaseLiftCommand command;
        if (!has_target_) return command;
        if (!lift_primed_) {
            // Cannot shape from an unknown start; wait for valid feedback.
            if (!feedback.position_valid) return command;
            shaper_->prime(feedback.position_mm);
            lift_primed_ = true;
        }
        shaper_->setTargetMm(target_.z_m * 1000.0);
        const LiftShaper::Step step = shaper_->step();
        if (!step.ok) {
            shaping_fault_ = "lift trajectory shaping failed (ruckig " +
                             std::to_string(step.ruckig_result) + ")";
            return command;
        }
        // The shaped target stopped moving and the drive confirmed arrival:
        // stop touching the SDO channel until the target changes.
        if (last_sent_position_mm_ &&
            std::abs(step.position_mm - *last_sent_position_mm_) <=
                kLiftResendEpsilonMm &&
            feedback.target_reached)
            return command;
        const double segment_speed_mm_s =
            std::max(std::abs(step.velocity_before_mm_s),
                     std::abs(step.velocity_after_mm_s));
        command.send = true;
        command.position_mm = step.position_mm;
        command.profile_velocity_mm_s =
            std::clamp(segment_speed_mm_s, kLiftProfileVelocityFloorMmS,
                       lift_profile_ceiling_mm_s_);
        last_sent_position_mm_ = step.position_mm;
        return command;
    }

    const bool chassis_served_;
    const bool lift_served_;
    const double lift_min_mm_;
    const double lift_max_mm_;
    const double lift_profile_ceiling_mm_s_;
    const int lift_divider_;
    const double lift_period_s_;

    std::unique_ptr<LiftShaper> shaper_;

    uint64_t cycle_index_{0};
    BaseTarget target_{};
    bool has_target_{false};
    bool chassis_target_dirty_{false};
    bool lift_primed_{false};
    std::optional<double> last_sent_position_mm_;
    std::string shaping_fault_;
};

// Compose the public BaseState from the chassis plugin state and the cached
// auxiliary snapshot. Absent DoFs keep zeroed pose fields and report
// unhealthy so callers cannot mistake them for live estimates.
inline BaseState composeBaseState(bool chassis_served,
                                  const ChassisControllerState& chassis,
                                  bool lift_served, const AuxiliaryState& aux,
                                  bool has_target, const BaseTarget& target,
                                  const std::string& extra_fault = {}) {
    BaseState state;
    if (chassis_served) {
        state.x_m = chassis.pose.x_m;
        state.y_m = chassis.pose.y_m;
        state.yaw_rad = chassis.pose.yaw_rad;
        state.vx_m_s = chassis.twist.vx_m_s;
        state.vy_m_s = chassis.twist.vy_m_s;
        state.omega_z_rad_s = chassis.twist.omega_z_rad_s;
        state.chassis_enabled = chassis.enabled;
        state.chassis_healthy = chassis.healthy;
    }
    if (lift_served) {
        state.z_m = aux.lift_position_mm / 1000.0;
        state.vz_m_s = aux.lift_velocity_mm_s / 1000.0;
        state.lift_enabled = aux.lift_enabled;
        state.lift_healthy = aux.lift_enabled && aux.lift_position_valid &&
                             aux.lift_error_code == 0;
    }
    state.has_target = has_target;
    state.target = target;
    auto append = [&state](const std::string& reason) {
        if (reason.empty()) return;
        if (!state.fault_reason.empty()) state.fault_reason += "; ";
        state.fault_reason += reason;
    };
    if (chassis_served) append(chassis.fault_reason);
    append(aux.fault_reason);
    append(extra_fault);
    return state;
}

}  // namespace poke::detail
