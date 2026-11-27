// Offline unit tests for the floating-base cycle logic (src/base_detail.hpp)
// and BaseController config validation. No CAN hardware: the real
// AuxiliaryController is never connected and the control loop is exercised
// through BaseCycleLogic with fabricated feedback.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "poke/base.hpp"
#include "base_detail.hpp"

namespace {

void expect(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}

poke::AuxiliaryConfig makeLiftAuxConfig() {
    poke::AuxiliaryConfig aux;
    aux.enable_lift = true;
    aux.lift_control_mode = poke::LiftControlMode::MetricProfilePosition;
    aux.lift_motor_counts_per_revolution = 10000.0;
    aux.lift_reduction_ratio = 7.0;
    aux.lift_screw_lead_mm_per_revolution = 4.0;
    aux.lift_min_position_mm = 0.0;
    aux.lift_max_position_mm = 300.0;
    aux.lift_max_profile_velocity_mm_s = 50.0;
    return aux;
}

poke::AuxiliaryConfig makeFullAuxConfig() {
    poke::AuxiliaryConfig aux = makeLiftAuxConfig();
    aux.enable_chassis = true;
    return aux;
}

poke::BaseConfig makeBaseConfig() {
    poke::BaseConfig base;
    base.control_rate_hz = 50.0;
    base.lift_command_rate_hz = 20.0;
    base.lift_max_speed_m_s = 0.02;
    base.lift_max_acceleration_m_s2 = 0.05;
    base.lift_max_jerk_m_s3 = 0.20;
    return base;
}

struct LiftWaypoint {
    uint64_t cycle{0};
    double position_mm{0.0};
    double profile_velocity_mm_s{0.0};
};

// Drive the logic for `cycles` with an ideal lift that teleports to each
// waypoint and always reports target_reached; returns the sent waypoints.
std::vector<LiftWaypoint> runCycles(poke::detail::BaseCycleLogic& logic,
                                    int cycles, double start_position_mm,
                                    bool target_reached,
                                    uint64_t cycle_offset = 0) {
    std::vector<LiftWaypoint> waypoints;
    double position_mm = start_position_mm;
    for (int i = 0; i < cycles; ++i) {
        poke::detail::BaseLiftFeedback feedback;
        feedback.position_valid = true;
        feedback.position_mm = position_mm;
        feedback.target_reached = target_reached;
        const auto decision = logic.onCycle(feedback);
        expect(decision.lift.send ? decision.lift_slot : true,
               "lift commands only on decimated lift slots");
        if (decision.lift.send) {
            waypoints.push_back({cycle_offset + static_cast<uint64_t>(i),
                                 decision.lift.position_mm,
                                 decision.lift.profile_velocity_mm_s});
            position_mm = decision.lift.position_mm;
        }
    }
    return waypoints;
}

void testZShaping() {
    const auto aux = makeLiftAuxConfig();
    const auto base = makeBaseConfig();
    poke::detail::BaseCycleLogic logic(aux, base);
    expect(logic.liftDivider() == 3, "50/20 Hz decimates by ceil -> 3");
    const double period_s = logic.liftPeriodS();
    expect(std::abs(period_s - 0.06) < 1e-12, "lift period is 60 ms");
    expect(1.0 / period_s <= base.lift_command_rate_hz + 1e-9,
           "realized lift cadence never exceeds lift_command_rate_hz");

    poke::BaseTarget target;
    target.z_m = 0.20;
    logic.setTarget(target);
    const auto waypoints = runCycles(logic, 1200, 0.0, true);
    expect(!waypoints.empty(), "step target produces lift waypoints");

    const double v_max_mm_s = base.lift_max_speed_m_s * 1000.0;
    const double a_max_mm_s2 = base.lift_max_acceleration_m_s2 * 1000.0;
    double previous_position = 0.0;
    double previous_velocity = 0.0;
    uint64_t previous_cycle = 0;
    bool first = true;
    for (const auto& wp : waypoints) {
        if (!first)
            expect(wp.cycle - previous_cycle >= 3,
                   "waypoint cadence never exceeds the decimated rate");
        const double velocity =
            (wp.position_mm - previous_position) / period_s;
        expect(std::abs(velocity) <= v_max_mm_s + 1e-6,
               "shaped z obeys the velocity limit");
        if (!first)
            expect(std::abs(velocity - previous_velocity) / period_s <=
                       a_max_mm_s2 + 1e-6,
                   "shaped z obeys the acceleration limit");
        expect(wp.position_mm >= previous_position - 1e-9,
               "monotone step response");
        expect(wp.profile_velocity_mm_s >=
                   poke::detail::kLiftProfileVelocityFloorMmS - 1e-12,
               "profile velocity respects the floor");
        expect(wp.profile_velocity_mm_s <=
                   aux.lift_max_profile_velocity_mm_s + 1e-9,
               "profile velocity respects the drive ceiling");
        previous_position = wp.position_mm;
        previous_velocity = velocity;
        previous_cycle = wp.cycle;
        first = false;
    }
    expect(std::abs(waypoints.back().position_mm - 200.0) < 1e-6,
           "final waypoint equals the target");
    expect(logic.shapingFault().empty(), "no shaping fault on a clean move");

    // Converged with target_reached: the SDO channel is left alone.
    const auto idle = runCycles(logic, 60, 200.0, true);
    expect(idle.empty(), "reached, unmoved target is not resent");
    // Drive not yet reporting reached: the last waypoint keeps being resent.
    const auto resent = runCycles(logic, 6, 200.0, false);
    expect(!resent.empty(), "unreached target is resent");
    expect(std::abs(resent.back().position_mm - 200.0) < 1e-6,
           "resent waypoint equals the target");
}

void testZTargetClamp() {
    poke::detail::BaseCycleLogic logic(makeLiftAuxConfig(), makeBaseConfig());
    poke::BaseTarget target;
    target.z_m = 10.0;  // far above lift_max_position_mm = 300 mm
    logic.setTarget(target);
    expect(std::abs(logic.target().z_m - 0.300) < 1e-12,
           "z target above lift_max clamps to the limit");
    const auto waypoints = runCycles(logic, 2000, 0.0, true);
    expect(std::abs(waypoints.back().position_mm - 300.0) < 1e-6,
           "clamped move converges on the limit, not the raw target");
    target.z_m = -1.0;
    logic.setTarget(target);
    expect(std::abs(logic.target().z_m) < 1e-12,
           "z target below lift_min clamps to the limit");
}

void testDecimation() {
    poke::detail::BaseCycleLogic logic(makeFullAuxConfig(), makeBaseConfig());
    poke::BaseTarget target;
    target.z_m = 0.05;
    logic.setTarget(target);
    int chassis_ticks = 0;
    int lift_slots = 0;
    for (int i = 0; i < 30; ++i) {
        poke::detail::BaseLiftFeedback feedback;
        feedback.position_valid = true;
        const auto decision = logic.onCycle(feedback);
        if (decision.tick_chassis) ++chassis_ticks;
        if (decision.lift_slot) ++lift_slots;
    }
    expect(chassis_ticks == 30, "chassis is ticked every cycle");
    expect(lift_slots == 10, "lift runs only on every third cycle");
}

void testLiftOnlyAndChassisOnly() {
    // Lift-only: chassis is never ticked, z pipeline still serves targets.
    poke::detail::BaseCycleLogic lift_only(makeLiftAuxConfig(),
                                           makeBaseConfig());
    poke::BaseTarget target;
    target.x_m = 1.0;
    target.z_m = 0.05;
    lift_only.setTarget(target);
    bool any_chassis = false;
    bool any_lift = false;
    double position_mm = 0.0;
    for (int i = 0; i < 30; ++i) {
        poke::detail::BaseLiftFeedback feedback;
        feedback.position_valid = true;
        feedback.position_mm = position_mm;
        feedback.target_reached = true;
        const auto decision = lift_only.onCycle(feedback);
        any_chassis |= decision.tick_chassis || decision.push_chassis_target;
        if (decision.lift.send) {
            any_lift = true;
            position_mm = decision.lift.position_mm;
        }
    }
    expect(!any_chassis, "absent chassis DoFs are never driven");
    expect(any_lift, "lift-only config still serves z");

    // Chassis-only: no lift slots, targets route to the plugin.
    poke::AuxiliaryConfig chassis_aux;
    chassis_aux.enable_chassis = true;
    poke::detail::BaseCycleLogic chassis_only(chassis_aux, makeBaseConfig());
    chassis_only.setTarget(target);
    const auto decision = chassis_only.onCycle({});
    expect(decision.tick_chassis, "chassis-only config ticks the plugin");
    expect(decision.push_chassis_target, "chassis-only config routes targets");
    expect(!decision.lift_slot && !decision.lift.send,
           "absent lift DoF is never driven");
}

void testTargetRouting() {
    poke::detail::BaseCycleLogic logic(makeFullAuxConfig(), makeBaseConfig());
    poke::BaseTarget target;
    target.x_m = 0.5;
    target.y_m = -0.25;
    target.z_m = 0.10;
    target.yaw_rad = 0.7;
    logic.setTarget(target);

    poke::detail::BaseLiftFeedback feedback;
    feedback.position_valid = true;
    auto decision = logic.onCycle(feedback);
    expect(decision.push_chassis_target,
           "new base target pushes a chassis target");
    expect(std::abs(decision.chassis_target.x_m - 0.5) < 1e-12 &&
               std::abs(decision.chassis_target.y_m + 0.25) < 1e-12 &&
               std::abs(decision.chassis_target.yaw_rad - 0.7) < 1e-12,
           "chassis target carries x, y, yaw");
    expect(decision.lift_slot && decision.lift.send,
           "new base target starts the z pipeline");
    expect(decision.lift.position_mm > 0.0 &&
               decision.lift.position_mm <= 100.0 + 1e-9,
           "first z waypoint moves toward the target");

    decision = logic.onCycle(feedback);
    expect(!decision.push_chassis_target,
           "unchanged target is pushed to the plugin only once");

    // stop() clears both pipelines.
    logic.clearTarget();
    expect(!logic.hasTarget(), "stop clears the base target");
    for (int i = 0; i < 12; ++i) {
        decision = logic.onCycle(feedback);
        expect(!decision.push_chassis_target && !decision.lift.send,
               "no chassis pushes or lift waypoints after stop");
    }

    // A fresh target after stop re-primes z from measured feedback.
    feedback.position_mm = 40.0;
    poke::BaseTarget resumed;
    resumed.z_m = 0.041;
    logic.setTarget(resumed);
    bool resumed_send = false;
    double resumed_position = 0.0;
    for (int i = 0; i < 3 && !resumed_send; ++i) {
        decision = logic.onCycle(feedback);
        resumed_send = decision.lift.send;
        resumed_position = decision.lift.position_mm;
    }
    expect(resumed_send, "z pipeline resumes after stop");
    expect(resumed_position > 40.0 && resumed_position <= 41.0 + 1e-9,
           "z shaping re-primes from measured position after stop");
}

void testUnprimedLiftWaits() {
    poke::detail::BaseCycleLogic logic(makeLiftAuxConfig(), makeBaseConfig());
    poke::BaseTarget target;
    target.z_m = 0.10;
    logic.setTarget(target);
    poke::detail::BaseLiftFeedback stale;  // position_valid = false
    for (int i = 0; i < 6; ++i)
        expect(!logic.onCycle(stale).lift.send,
               "no waypoints while lift feedback is invalid");
    poke::detail::BaseLiftFeedback fresh;
    fresh.position_valid = true;
    fresh.position_mm = 20.0;
    bool sent = false;
    for (int i = 0; i < 3; ++i) sent |= logic.onCycle(fresh).lift.send;
    expect(sent, "shaping starts once feedback becomes valid");
}

// Default rates are 500 Hz for both the control thread and the lift cadence
// (divider 1, 2 ms shaping period). The ceiling-divider decimation stays in
// place so a config can dial lift_command_rate_hz back down (required on the
// deployed ~20 Hz SDO adapter, see BaseConfig).
void testDefaultRates() {
    const poke::BaseConfig defaults;
    expect(defaults.control_rate_hz == 500.0,
           "default base control rate is 500 Hz");
    expect(defaults.lift_command_rate_hz == 500.0,
           "default lift command rate is 500 Hz");
    expect(poke::detail::validateBaseConfig(makeFullAuxConfig(), defaults)
               .isOk(),
           "default 500/500 Hz config validates");

    poke::detail::BaseCycleLogic logic(makeLiftAuxConfig(), defaults);
    expect(logic.liftDivider() == 1, "500/500 Hz decimates by ceil -> 1");
    expect(std::abs(logic.liftPeriodS() - 0.002) < 1e-15,
           "lift shaping period is 2 ms at 500 Hz");
    poke::BaseTarget target;
    target.z_m = 0.01;
    logic.setTarget(target);
    for (int i = 0; i < 10; ++i) {
        poke::detail::BaseLiftFeedback feedback;
        feedback.position_valid = true;
        expect(logic.onCycle(feedback).lift_slot,
               "divider 1 makes every cycle a lift slot");
    }

    // The 2 ms-period shaper still respects the metric limits and converges.
    poke::detail::BaseCycleLogic shaped(makeLiftAuxConfig(), defaults);
    shaped.setTarget(target);
    const auto waypoints = runCycles(shaped, 1500, 0.0, true);
    expect(!waypoints.empty(), "500 Hz shaping produces waypoints");
    const double v_max_mm_s = defaults.lift_max_speed_m_s * 1000.0;
    double previous_position = 0.0;
    uint64_t previous_cycle = 0;
    bool first = true;
    for (const auto& wp : waypoints) {
        if (!first)
            expect(std::abs(wp.position_mm - previous_position) /
                       (static_cast<double>(wp.cycle - previous_cycle) *
                        logic.liftPeriodS()) <= v_max_mm_s + 1e-6,
                   "500 Hz shaped z obeys the velocity limit");
        previous_position = wp.position_mm;
        previous_cycle = wp.cycle;
        first = false;
    }
    expect(std::abs(waypoints.back().position_mm - 10.0) < 1e-6,
           "500 Hz shaping converges on the target");
    expect(shaped.shapingFault().empty(), "no shaping fault at 500 Hz");

    // Decimation survives the rate bump: 500/20 Hz yields divider 25.
    poke::BaseConfig decimated = defaults;
    decimated.lift_command_rate_hz = 20.0;
    poke::detail::BaseCycleLogic slow(makeLiftAuxConfig(), decimated);
    expect(slow.liftDivider() == 25, "500/20 Hz decimates by ceil -> 25");
    expect(std::abs(slow.liftPeriodS() - 0.05) < 1e-12,
           "decimated lift period is 50 ms");
}

void testConfigValidation() {
    using poke::detail::validateBaseConfig;
    expect(validateBaseConfig(makeFullAuxConfig(), makeBaseConfig()).isOk(),
           "nominal chassis+lift config validates");
    expect(validateBaseConfig(makeLiftAuxConfig(), makeBaseConfig()).isOk(),
           "lift-only config validates");

    poke::AuxiliaryConfig none;
    expect(validateBaseConfig(none, makeBaseConfig()).code() ==
               poke::ErrorCode::InvalidConfig,
           "no DoF at all is rejected");

    // Lift enabled without the metric Profile Position mode/parameters.
    poke::AuxiliaryConfig raw = makeLiftAuxConfig();
    raw.lift_control_mode = poke::LiftControlMode::RawProfileVelocity;
    expect(validateBaseConfig(raw, makeBaseConfig()).code() ==
               poke::ErrorCode::InvalidConfig,
           "raw lift mode is rejected");
    poke::AuxiliaryConfig no_metric = makeLiftAuxConfig();
    no_metric.lift_screw_lead_mm_per_revolution = 0.0;
    expect(validateBaseConfig(no_metric, makeBaseConfig()).code() ==
               poke::ErrorCode::InvalidConfig,
           "missing metric lift parameters are rejected");

    poke::BaseConfig fast = makeBaseConfig();
    fast.lift_command_rate_hz = 100.0;  // above control_rate_hz
    expect(validateBaseConfig(makeLiftAuxConfig(), fast).code() ==
               poke::ErrorCode::InvalidConfig,
           "lift cadence above the control rate is rejected");

    // Unknown chassis plugin type: the factory returns nullptr and the
    // facade surfaces it as a config error before touching hardware.
    poke::BaseConfig bogus = makeBaseConfig();
    bogus.chassis_controller = "no_such_controller";
    poke::BaseController controller(makeFullAuxConfig(), bogus);
    const auto connect_status = controller.connect();
    expect(connect_status.code() == poke::ErrorCode::InvalidConfig,
           "unknown chassis controller type is a config error");
    expect(controller.enable().code() == poke::ErrorCode::InvalidConfig,
           "enable also surfaces the chassis plugin config error");

    poke::BaseController bad_lift(raw, makeBaseConfig());
    expect(bad_lift.connect().code() == poke::ErrorCode::InvalidConfig,
           "BaseController rejects a non-metric lift before connecting");
}

void testBaseStateComposition() {
    poke::ChassisControllerState chassis;
    chassis.enabled = true;
    chassis.healthy = true;
    chassis.pose = {1.5, -0.5, 0.3};
    chassis.twist = {0.1, -0.05, 0.02};

    poke::AuxiliaryState aux;
    aux.lift_enabled = true;
    aux.lift_position_valid = true;
    aux.lift_position_mm = 120.0;
    aux.lift_velocity_mm_s = -8.0;

    poke::BaseTarget target;
    target.x_m = 2.0;
    target.z_m = 0.15;

    const auto state = poke::detail::composeBaseState(
        true, chassis, true, aux, true, target);
    expect(std::abs(state.x_m - 1.5) < 1e-12 &&
               std::abs(state.y_m + 0.5) < 1e-12 &&
               std::abs(state.yaw_rad - 0.3) < 1e-12,
           "planar pose comes from the chassis plugin");
    expect(std::abs(state.z_m - 0.120) < 1e-12,
           "z_m is lift_position_mm / 1000");
    expect(std::abs(state.vz_m_s + 0.008) < 1e-12,
           "vz_m_s is lift_velocity_mm_s / 1000");
    expect(std::abs(state.vx_m_s - 0.1) < 1e-12 &&
               std::abs(state.omega_z_rad_s - 0.02) < 1e-12,
           "twist comes from the chassis plugin");
    expect(state.chassis_enabled && state.chassis_healthy &&
               state.lift_enabled && state.lift_healthy,
           "health flags compose from both layers");
    expect(state.has_target && std::abs(state.target.x_m - 2.0) < 1e-12,
           "target echoes through the composed state");
    expect(state.fault_reason.empty(), "healthy layers report no fault");

    // Absent chassis DoF: planar fields stay zeroed and unhealthy.
    const auto lift_only = poke::detail::composeBaseState(
        false, chassis, true, aux, false, {});
    expect(lift_only.x_m == 0.0 && !lift_only.chassis_healthy &&
               !lift_only.chassis_enabled,
           "absent chassis reports zero pose and unhealthy");
    expect(std::abs(lift_only.z_m - 0.120) < 1e-12,
           "lift-only state still serves z");

    // Faults from either layer aggregate.
    chassis.fault_reason = "wheel 2 stale";
    aux.fault_reason = "lift SDO timeout";
    aux.lift_error_code = 0x8611;
    const auto faulted = poke::detail::composeBaseState(
        true, chassis, true, aux, false, {}, "loop overrun");
    expect(!faulted.lift_healthy, "drive error code marks the lift unhealthy");
    expect(faulted.fault_reason.find("wheel 2 stale") != std::string::npos &&
               faulted.fault_reason.find("lift SDO timeout") !=
                   std::string::npos &&
               faulted.fault_reason.find("loop overrun") != std::string::npos,
           "fault reasons aggregate across layers");
}

}  // namespace

int main() {
    testZShaping();
    testZTargetClamp();
    testDecimation();
    testLiftOnlyAndChassisOnly();
    testTargetRouting();
    testUnprimedLiftWaits();
    testDefaultRates();
    testConfigValidation();
    testBaseStateComposition();
    std::cout << "base tests passed\n";
    return 0;
}
