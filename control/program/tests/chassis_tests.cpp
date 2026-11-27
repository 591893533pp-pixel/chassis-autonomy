// Offline tests for the "xdrive_odom" chassis-controller plugin. No CAN
// hardware: the plugin is driven through a fake ChassisWheelInterface whose
// feedback either echoes the last command (perfect wheels) or replays scripted
// wheel velocities (odometry checks).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "poke/chassis_controller.hpp"

namespace {

void expect(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}

struct FakeWheels {
    // Feedback model: when follow_commands is true, measured velocity is the
    // last commanded velocity scaled per wheel (response_gain), i.e. motors
    // that track their internal velocity mode perfectly or with a fixed lag.
    bool follow_commands{true};
    std::array<double, 4> response_gain{{1.0, 1.0, 1.0, 1.0}};
    std::array<double, 4> feedback{};
    bool feedback_fresh{true};
    // Wall-clock age reported alongside a fresh sample (the io layer serves
    // cached type-24 reports, so "fresh" samples carry a nonzero age).
    double age_ms{5.0};
    bool fail_send{false};
    int stop_calls{0};
    std::array<double, 4> last_sent{};
    std::vector<std::array<double, 4>> sent_history;

    poke::ChassisWheelInterface interface() {
        poke::ChassisWheelInterface io;
        io.send_wheel_velocities = [this](const std::array<double, 4>& wheels) {
            if (fail_send)
                return poke::Status(poke::ErrorCode::TransportError,
                                    "injected send failure");
            last_sent = wheels;
            sent_history.push_back(wheels);
            if (follow_commands)
                for (std::size_t i = 0; i < wheels.size(); ++i)
                    feedback[i] = response_gain[i] * wheels[i];
            return poke::Status::ok();
        };
        io.stop_wheels = [this]() {
            ++stop_calls;
            last_sent.fill(0.0);
            if (follow_commands) feedback.fill(0.0);
            return poke::Status::ok();
        };
        io.measured_wheel_velocities = [this](std::array<double, 4>& wheels,
                                              double& sample_age_ms) {
            if (!feedback_fresh) return false;
            wheels = feedback;
            sample_age_ms = age_ms;
            return true;
        };
        return io;
    }
};

constexpr double kDt = 0.02;  // matches the default 50 Hz control rate

poke::ChassisTwist impliedTwist(const poke::XDriveKinematics& kinematics,
                                const std::array<double, 4>& wheels) {
    return kinematics.forward(wheels);
}

void testKinematics() {
    poke::XDriveKinematics kinematics(0.0762, 0.25213, 9.424777960769);

    // Round trip: inverse then forward reproduces the applied twist.
    const poke::ChassisTwist twist{0.3, -0.2, 0.1};
    const auto solution = kinematics.inverse(twist);
    expect(solution.scale == 1.0, "small twist is not scaled");
    const auto recovered = kinematics.forward(solution.wheel_rad_s);
    expect(std::abs(recovered.vx_m_s - twist.vx_m_s) < 1e-12 &&
           std::abs(recovered.vy_m_s - twist.vy_m_s) < 1e-12 &&
           std::abs(recovered.omega_z_rad_s - twist.omega_z_rad_s) < 1e-12,
           "inverse/forward kinematics round trip");

    // Wheel-cap saturation scales all wheels uniformly: the fastest wheel
    // sits exactly at the cap and ratios (i.e. the motion direction) hold.
    const poke::ChassisTwist fast{2.0, -1.5, 0.3};
    const auto capped = kinematics.inverse(fast);
    expect(capped.scale < 1.0, "over-limit twist is scaled down");
    double peak = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        peak = std::max(peak, std::abs(capped.wheel_rad_s[i]));
        expect(std::abs(capped.wheel_rad_s[i] -
                        capped.scale * capped.unscaled_rad_s[i]) < 1e-12,
               "wheel scaling is uniform");
    }
    expect(std::abs(peak - 9.424777960769) < 1e-9,
           "fastest wheel sits exactly at the cap");
    const auto applied = kinematics.forward(capped.wheel_rad_s);
    expect(std::abs(applied.vx_m_s - capped.applied_twist.vx_m_s) < 1e-12 &&
           std::abs(applied.vy_m_s - capped.applied_twist.vy_m_s) < 1e-12 &&
           std::abs(applied.omega_z_rad_s -
                    capped.applied_twist.omega_z_rad_s) < 1e-12,
           "applied twist matches capped wheels");
    expect(std::abs(capped.applied_twist.vx_m_s - capped.scale * fast.vx_m_s) < 1e-12,
           "applied twist is the uniformly scaled request");
}

void testFactory() {
    FakeWheels wheels;
    poke::XDriveOdomConfig config;
    expect(poke::makeChassisController("unknown", config, wheels.interface()) ==
           nullptr, "unknown controller type yields nullptr");
    expect(poke::makeChassisController("xdrive_odom", config,
                                       wheels.interface()) != nullptr,
           "xdrive_odom type is supported");
    poke::XDriveOdomConfig bad = config;
    bad.wheel_radius_m = -1.0;
    expect(poke::makeChassisController("xdrive_odom", bad,
                                       wheels.interface()) == nullptr,
           "invalid geometry yields nullptr instead of throwing");

    // enable() requires a complete wheel interface.
    auto controller = poke::makeChassisController(
        "xdrive_odom", config, poke::ChassisWheelInterface{});
    expect(controller != nullptr, "controller constructs without io");
    expect(!controller->enable(), "enable rejects an incomplete interface");
    expect(!controller->tick(kDt), "tick rejected while disabled");
}

void testOdometry() {
    FakeWheels wheels;
    wheels.follow_commands = false;  // scripted feedback, ignore commands
    poke::XDriveOdomConfig config;
    poke::XDriveKinematics kinematics(config.wheel_radius_m,
                                      config.wheel_center_radius_m,
                                      config.max_wheel_speed_rad_s);
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "odometry enable");

    const auto drive = [&](const poke::ChassisTwist& twist, int ticks) {
        wheels.feedback = kinematics.inverse(twist).wheel_rad_s;
        for (int i = 0; i < ticks; ++i)
            expect(static_cast<bool>(controller->tick(kDt)), "odometry tick");
    };

    // Square path at yaw 0: 1 s per side at 0.2 m/s, back to the origin.
    drive({0.2, 0.0, 0.0}, 50);
    auto pose = controller->state().pose;
    expect(std::abs(pose.x_m - 0.2) < 1e-9 && std::abs(pose.y_m) < 1e-9,
           "odometry +x leg");
    drive({0.0, 0.2, 0.0}, 50);
    drive({-0.2, 0.0, 0.0}, 50);
    drive({0.0, -0.2, 0.0}, 50);
    pose = controller->state().pose;
    expect(std::abs(pose.x_m) < 1e-9 && std::abs(pose.y_m) < 1e-9 &&
           std::abs(pose.yaw_rad) < 1e-9, "odometry square closes");

    // Pure rotation, then a body-frame +x leg that must land world +y.
    drive({0.0, 0.0, 0.157079632679}, 500);  // 10 s -> pi/2
    pose = controller->state().pose;
    expect(std::abs(pose.yaw_rad - 1.5707963267948966) < 1e-9,
           "odometry pure rotation");
    drive({0.2, 0.0, 0.0}, 50);
    pose = controller->state().pose;
    expect(std::abs(pose.x_m) < 1e-9 && std::abs(pose.y_m - 0.2) < 1e-9,
           "rotated body twist integrates into world frame");

    // Stale feedback must hold the pose even though wheels report motion...
    wheels.feedback_fresh = false;
    wheels.feedback = kinematics.inverse({0.5, 0.0, 0.0}).wheel_rad_s;
    for (int i = 0; i < 50; ++i) (void)controller->tick(kDt);
    const auto held = controller->state().pose;
    expect(std::abs(held.x_m - pose.x_m) < 1e-12 &&
           std::abs(held.y_m - pose.y_m) < 1e-12,
           "stale feedback does not integrate");
    expect(!controller->state().healthy,
           "persistently stale feedback reports unhealthy");
    // ...and commanded velocities must not be dead-reckoned either.
    wheels.feedback_fresh = true;
    expect(static_cast<bool>(controller->tick(kDt)),
           "recovers after feedback returns");
    expect(controller->state().healthy, "healthy again with fresh feedback");
}

void testTargetMode() {
    FakeWheels wheels;  // perfect wheels: feedback = last command
    poke::XDriveOdomConfig config;
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "target enable");

    const poke::ChassisTarget goal{0.3, 0.1, 0.2};
    bool reached = false;
    for (int i = 0; i < 1500; ++i) {
        // A real client streams its target; re-issuing keeps the watchdog fed.
        if (i % 10 == 0)
            expect(static_cast<bool>(controller->setTarget(goal)), "setTarget");
        expect(static_cast<bool>(controller->tick(kDt)), "target tick");
        const auto pose = controller->state().pose;
        const double translation_error =
            std::hypot(goal.x_m - pose.x_m, goal.y_m - pose.y_m);
        const double yaw_error = std::abs(goal.yaw_rad - pose.yaw_rad);
        const bool inside = translation_error <= config.translation_tolerance_m &&
                            yaw_error <= config.yaw_tolerance_rad;
        if (inside) reached = true;
        if (reached)
            // Inside tolerance the loop zeroes its twist; the residual ramp-
            // down must not push the pose back out (no overshoot).
            expect(translation_error <= 1.5 * config.translation_tolerance_m &&
                   yaw_error <= 1.5 * config.yaw_tolerance_rad,
                   "no overshoot beyond tolerance");
    }
    expect(reached, "target mode converges");
    expect(controller->state().has_target, "target still held");
    const auto commanded = controller->state().commanded;
    expect(std::abs(commanded.vx_m_s) < 1e-6 &&
           std::abs(commanded.vy_m_s) < 1e-6 &&
           std::abs(commanded.omega_z_rad_s) < 1e-6,
           "commanded twist settles to zero at the target");
}

void testSetCurrentAsOrigin() {
    FakeWheels wheels;
    wheels.follow_commands = false;
    poke::XDriveOdomConfig config;
    poke::XDriveKinematics kinematics(config.wheel_radius_m,
                                      config.wheel_center_radius_m,
                                      config.max_wheel_speed_rad_s);
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "origin enable");

    // Move the odometry to (0.1, 0, 0) with scripted feedback.
    wheels.feedback = kinematics.inverse({0.2, 0.0, 0.0}).wheel_rad_s;
    for (int i = 0; i < 25; ++i) (void)controller->tick(kDt);
    wheels.feedback.fill(0.0);
    (void)controller->setTarget({0.3, 0.1, 0.2});
    expect(static_cast<bool>(controller->setCurrentAsOrigin()),
           "setCurrentAsOrigin");
    const auto state = controller->state();
    expect(std::abs(state.pose.x_m) < 1e-12 && std::abs(state.pose.y_m) < 1e-12 &&
           std::abs(state.pose.yaw_rad) < 1e-12, "origin re-zeroed");
    // The physical goal survives the re-anchor: it is re-expressed relative
    // to the old pose (0.1, 0, 0), so the tracking error does not jump.
    expect(state.has_target &&
           std::abs(state.target.x_m - 0.2) < 1e-9 &&
           std::abs(state.target.y_m - 0.1) < 1e-9 &&
           std::abs(state.target.yaw_rad - 0.2) < 1e-9,
           "target re-expressed in the new frame");
}

void testTwistRamp() {
    FakeWheels wheels;
    poke::XDriveOdomConfig config;
    config.velocity_balance_enabled = false;  // isolate the shaping stage
    poke::XDriveKinematics kinematics(config.wheel_radius_m,
                                      config.wheel_center_radius_m,
                                      config.max_wheel_speed_rad_s);
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "twist enable");

    double previous_vx = 0.0;
    for (int i = 0; i < 300; ++i) {
        expect(static_cast<bool>(controller->setTwist({0.5, 0.0, 0.0})),
               "setTwist");
        expect(static_cast<bool>(controller->tick(kDt)), "twist tick");
        const double vx = impliedTwist(kinematics, wheels.last_sent).vx_m_s;
        expect(vx - previous_vx <=
               config.max_linear_acceleration_m_s2 * kDt + 1e-9,
               "twist ramp respects the acceleration limit");
        expect(vx >= previous_vx - 1e-9, "ramp to a higher speed is monotone");
        previous_vx = vx;
    }
    expect(std::abs(previous_vx - 0.5) < 1e-6, "ramp reaches the commanded twist");

    // The linear cap scales the vector uniformly: an over-limit diagonal
    // request keeps its direction instead of clipping x and y separately.
    for (int i = 0; i < 2000; ++i) {
        (void)controller->setTwist({2.0, 1.0, 0.0});
        expect(static_cast<bool>(controller->tick(kDt)), "capped twist tick");
    }
    const auto capped = impliedTwist(kinematics, wheels.last_sent);
    expect(std::abs(capped.vx_m_s / capped.vy_m_s - 2.0) < 1e-6,
           "linear cap preserves the commanded direction");
    expect(std::hypot(capped.vx_m_s, capped.vy_m_s) <=
           config.max_linear_speed_m_s + 1e-6,
           "linear cap bounds the speed vector norm");
}

void testWatchdog() {
    FakeWheels wheels;
    poke::XDriveOdomConfig config;
    config.velocity_balance_enabled = false;
    poke::XDriveKinematics kinematics(config.wheel_radius_m,
                                      config.wheel_center_radius_m,
                                      config.max_wheel_speed_rad_s);
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "watchdog enable");

    // Ramp up while the command stream is alive.
    for (int i = 0; i < 300; ++i) {
        (void)controller->setTwist({0.4, 0.0, 0.0});
        (void)controller->tick(kDt);
    }
    expect(std::abs(impliedTwist(kinematics, wheels.last_sent).vx_m_s - 0.4) <
           1e-6, "watchdog test reaches cruise");

    // Stop feeding commands. Within the watchdog window the last twist holds;
    // afterwards the controller must ramp to zero (accel-limited) and the
    // zero command must actually be sent every tick.
    const int window_ticks =
        static_cast<int>(config.command_watchdog_ms / 1000.0 / kDt);
    double previous_vx = 0.4;
    for (int i = 0; i < window_ticks - 2; ++i) {
        (void)controller->tick(kDt);
        previous_vx = impliedTwist(kinematics, wheels.last_sent).vx_m_s;
    }
    expect(std::abs(previous_vx - 0.4) < 1e-6,
           "command holds inside the watchdog window");
    const int ramp_ticks = static_cast<int>(
        (0.4 / config.max_linear_acceleration_m_s2 + 1.0) / kDt) + 50;
    for (int i = 0; i < ramp_ticks + 10; ++i) {
        (void)controller->tick(kDt);
        const double vx = impliedTwist(kinematics, wheels.last_sent).vx_m_s;
        expect(previous_vx - vx <=
               config.max_linear_acceleration_m_s2 * kDt + 1e-9,
               "watchdog ramp respects the acceleration limit");
        previous_vx = vx;
    }
    for (const double wheel : wheels.last_sent)
        expect(std::abs(wheel) < 1e-6, "watchdog zero reaches the wheels");
    expect(!controller->state().has_target, "watchdog drops the command");
    expect(!controller->state().fault_reason.empty(),
           "watchdog records its reason");

    // stop() must also ramp down and keep sending zeros.
    (void)controller->setTwist({0.3, 0.0, 0.0});
    for (int i = 0; i < 100; ++i) {
        (void)controller->setTwist({0.3, 0.0, 0.0});
        (void)controller->tick(kDt);
    }
    (void)controller->stop();
    for (int i = 0; i < ramp_ticks; ++i) (void)controller->tick(kDt);
    for (const double wheel : wheels.last_sent)
        expect(std::abs(wheel) < 1e-6, "stop ramps the wheels to zero");
}

void testBalanceTrim() {
    FakeWheels wheels;
    wheels.response_gain[1] = 0.9;  // wheel 2 runs 10% slow
    poke::XDriveOdomConfig config;
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "balance enable");

    for (int i = 0; i < 500; ++i) {
        (void)controller->setTwist({0.2, 0.0, 0.0});
        expect(static_cast<bool>(controller->tick(kDt)), "balance tick");
        // For pure +vx all four raw wheel targets share one magnitude, and
        // wheel 2 (gain 1.0, same sign as wheel 1) carries no trim, so its
        // command exposes wheel 1's raw target.
        const double raw = wheels.last_sent[2];
        if (std::abs(raw) < 1e-6) continue;
        const double trimmed = wheels.last_sent[1];
        expect(trimmed >= 0.0, "trim never flips the wheel sign");
        expect(trimmed <=
               raw * (1.0 + config.velocity_balance_max_fraction) + 1e-9,
               "trim never exceeds balance_max_fraction");
    }
    // Steady state: the integrator lifts the slow wheel to raw/0.9 (an 11%
    // boost, inside the 25% authority), so its measured speed matches.
    const double raw = wheels.last_sent[2];
    const double trimmed = wheels.last_sent[1];
    expect(trimmed > raw * 1.05, "trim pushes the slow wheel up");
    expect(std::abs(trimmed - raw / 0.9) < 0.02 * raw,
           "integrator settles at the exact compensation");

    // A reversal must restart the integrator: through the ramp and after the
    // sign flip the trim keeps tracking the raw target's sign and stays
    // bounded by the fresh target's authority.
    for (int i = 0; i < 500; ++i) {
        (void)controller->setTwist({-0.2, 0.0, 0.0});
        (void)controller->tick(kDt);
        const double reversed_raw = wheels.last_sent[2];
        if (std::abs(reversed_raw) < 1e-6) continue;
        expect(wheels.last_sent[1] * reversed_raw >= 0.0,
               "trim never opposes the raw target through the reversal");
        // The reference wheel carries its own (possibly negative) trim while
        // decelerating, so the observable bound is (1+f)/(1-f) of its command.
        const double bound_ratio =
            (1.0 + config.velocity_balance_max_fraction) /
            (1.0 - config.velocity_balance_max_fraction);
        expect(std::abs(wheels.last_sent[1]) <=
               std::abs(reversed_raw) * bound_ratio + 1e-9,
               "trim stays bounded through the reversal");
    }
    expect(wheels.last_sent[1] < 0.0, "reversed wheel command settles negative");
}

// The stale-feedback fault criterion is wall-clock feedback age, not "no new
// sample this tick": at 500 Hz (2 ms) the 100 Hz type-24 reports naturally
// refresh only every ~5th tick and the cached samples in between must stay
// healthy, while feedback older than ~3 report periods (30 ms) is a fault.
void testStaleFeedbackAging() {
    FakeWheels wheels;
    wheels.follow_commands = false;  // scripted feedback
    poke::XDriveOdomConfig config;
    config.control_rate_hz = 500.0;
    const double dt = 1.0 / config.control_rate_hz;
    poke::XDriveKinematics kinematics(config.wheel_radius_m,
                                      config.wheel_center_radius_m,
                                      config.max_wheel_speed_rad_s);
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "aging enable");

    // 1 s of +x driving where the cached sample's age cycles 0..8 ms, i.e.
    // a new report only every 5th tick: never a fault, and odometry keeps
    // integrating the cached samples every tick.
    wheels.feedback = kinematics.inverse({0.2, 0.0, 0.0}).wheel_rad_s;
    for (int i = 0; i < 500; ++i) {
        wheels.age_ms = 2.0 * static_cast<double>(i % 5);
        expect(static_cast<bool>(controller->tick(dt)), "aging tick");
        expect(controller->state().healthy,
               "sub-period feedback age stays healthy at 500 Hz");
    }
    expect(std::abs(controller->state().pose.x_m - 0.2) < 1e-9,
           "cached samples integrate into odometry every tick");

    // A sample that is valid at the io layer but older than the fault bound
    // reports unhealthy (and says why) while remaining usable.
    wheels.age_ms = 40.0;
    expect(static_cast<bool>(controller->tick(dt)), "over-age tick");
    expect(!controller->state().healthy, "over-age feedback reports unhealthy");
    expect(controller->state().fault_reason.find("stale") != std::string::npos,
           "over-age feedback records a stale fault reason");

    // Health recovers as soon as a fresh-enough sample arrives.
    wheels.age_ms = 4.0;
    expect(static_cast<bool>(controller->tick(dt)), "recovery tick");
    expect(controller->state().healthy, "young feedback restores health");

    // Rejected samples age by wall-clock dt: from age 4 ms the fault lands
    // once the accumulated age passes 30 ms (~13-15 ticks at 2 ms), not
    // after a fixed tick count.
    wheels.feedback_fresh = false;
    int ticks_to_fault = 0;
    while (controller->state().healthy && ticks_to_fault < 100) {
        (void)controller->tick(dt);
        ++ticks_to_fault;
    }
    expect(ticks_to_fault >= 13 && ticks_to_fault <= 15,
           "rejected feedback faults after ~30 ms of accumulated age");
    for (const double wheel : wheels.last_sent)
        expect(std::abs(wheel) < 1e-12,
               "wheels hold zero while feedback is rejected");

    // Equivalence with the 50 Hz deployment: the same wall-clock bound means
    // a couple of 20 ms ticks without a sample, exactly the old 3-tick fault
    // latency ballpark, and never a fault while 10 ms reports keep arriving.
    poke::XDriveOdomConfig slow = config;
    slow.control_rate_hz = 50.0;
    FakeWheels slow_wheels;
    slow_wheels.follow_commands = false;
    auto slow_controller = poke::makeChassisController("xdrive_odom", slow,
                                                       slow_wheels.interface());
    expect(static_cast<bool>(slow_controller->enable()), "50 Hz aging enable");
    for (int i = 0; i < 100; ++i) {
        slow_wheels.age_ms = 5.0;
        (void)slow_controller->tick(kDt);
        expect(slow_controller->state().healthy, "50 Hz cadence stays healthy");
    }
    slow_wheels.feedback_fresh = false;
    int slow_ticks = 0;
    while (slow_controller->state().healthy && slow_ticks < 10) {
        (void)slow_controller->tick(kDt);
        ++slow_ticks;
    }
    expect(slow_ticks == 2,
           "50 Hz fault latency stays in the old few-tick ballpark");
}

// The odometry integration cap must not shrink with the control rate: at
// 500 Hz a lift-slot cycle blocks on SDO traffic for tens of milliseconds
// and the (still valid) cached wheel sample has to cover that whole cycle.
// The cap stays exactly the deployed 40 ms at 50 Hz.
void testOdometryStepCapIsRateAgnostic() {
    for (const double rate_hz : {500.0, 50.0}) {
        FakeWheels wheels;
        wheels.follow_commands = false;
        poke::XDriveOdomConfig config;
        config.control_rate_hz = rate_hz;
        poke::XDriveKinematics kinematics(config.wheel_radius_m,
                                          config.wheel_center_radius_m,
                                          config.max_wheel_speed_rad_s);
        auto controller = poke::makeChassisController("xdrive_odom", config,
                                                      wheels.interface());
        expect(static_cast<bool>(controller->enable()), "step-cap enable");
        wheels.feedback = kinematics.inverse({0.2, 0.0, 0.0}).wheel_rad_s;
        wheels.age_ms = 5.0;

        // A 30 ms cycle (lift SDO block) integrates the full 30 ms.
        expect(static_cast<bool>(controller->tick(0.030)), "step-cap 30 ms tick");
        expect(std::abs(controller->state().pose.x_m - 0.2 * 0.030) < 1e-9,
               "a 30 ms cycle integrates 30 ms of travel at any rate");

        // A gross stall is still clamped: 500 ms integrates only 40 ms.
        expect(static_cast<bool>(controller->tick(0.500)), "step-cap 500 ms tick");
        expect(std::abs(controller->state().pose.x_m - 0.2 * (0.030 + 0.040)) < 1e-9,
               "a 500 ms stall is clamped to the 40 ms cap");

        // Normal cycles are untouched.
        const double dt = 1.0 / rate_hz;
        expect(static_cast<bool>(controller->tick(dt)), "step-cap nominal tick");
        expect(std::abs(controller->state().pose.x_m -
                        0.2 * (0.030 + 0.040 + dt)) < 1e-9,
               "nominal cycles integrate their own dt");
    }
}

void testSendFailure() {
    FakeWheels wheels;
    poke::XDriveOdomConfig config;
    auto controller = poke::makeChassisController("xdrive_odom", config,
                                                  wheels.interface());
    expect(static_cast<bool>(controller->enable()), "send-failure enable");
    (void)controller->setTwist({0.2, 0.0, 0.0});
    for (int i = 0; i < 20; ++i)
        expect(static_cast<bool>(controller->tick(kDt)), "healthy tick");
    expect(controller->state().healthy, "healthy before the failure");

    const int stops_before = wheels.stop_calls;
    wheels.fail_send = true;
    const auto status = controller->tick(kDt);
    expect(!status && status.code() == poke::ErrorCode::TransportError,
           "send failure surfaces from tick");
    const auto state = controller->state();
    expect(!state.healthy, "send failure marks unhealthy");
    expect(!state.fault_reason.empty(), "send failure records fault_reason");
    expect(wheels.stop_calls == stops_before + 1,
           "send failure attempts stop_wheels");
}

}  // namespace

int main() {
    testKinematics();
    testFactory();
    testOdometry();
    testTargetMode();
    testSetCurrentAsOrigin();
    testTwistRamp();
    testWatchdog();
    testBalanceTrim();
    testStaleFeedbackAging();
    testOdometryStepCapIsRateAgnostic();
    testSendFailure();
    std::cout << "chassis tests passed\n";
    return 0;
}
