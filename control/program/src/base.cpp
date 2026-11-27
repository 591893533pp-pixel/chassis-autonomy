#include "poke/base.hpp"

#include "auxiliary.hpp"
#include "base_detail.hpp"

#include <ruckig/ruckig.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <utility>

namespace poke {

namespace {

using Clock = std::chrono::steady_clock;

}  // namespace

namespace detail {

struct LiftShaper::Impl {
    explicit Impl(double period_s) : otg(period_s) {}
    ruckig::Ruckig<1> otg;
    ruckig::InputParameter<1> input;
    ruckig::OutputParameter<1> output;
};

LiftShaper::LiftShaper(double period_s, double max_velocity_mm_s,
                       double max_acceleration_mm_s2, double max_jerk_mm_s3)
    : impl_(std::make_unique<Impl>(period_s)) {
    impl_->input.control_interface = ruckig::ControlInterface::Position;
    impl_->input.max_velocity = {max_velocity_mm_s};
    impl_->input.max_acceleration = {max_acceleration_mm_s2};
    impl_->input.max_jerk = {max_jerk_mm_s3};
    impl_->input.target_velocity = {0.0};
    impl_->input.target_acceleration = {0.0};
}

LiftShaper::~LiftShaper() = default;

void LiftShaper::prime(double position_mm) {
    impl_->input.current_position = {position_mm};
    impl_->input.current_velocity = {0.0};
    impl_->input.current_acceleration = {0.0};
}

void LiftShaper::setTargetMm(double target_mm) {
    impl_->input.target_position = {target_mm};
}

LiftShaper::Step LiftShaper::step() {
    Step step;
    step.velocity_before_mm_s = impl_->input.current_velocity[0];
    const auto result = impl_->otg.update(impl_->input, impl_->output);
    step.ruckig_result = static_cast<int>(result);
    if (result != ruckig::Result::Working &&
        result != ruckig::Result::Finished)
        return step;
    impl_->output.pass_to_input(impl_->input);
    step.ok = true;
    step.position_mm = impl_->output.new_position[0];
    step.velocity_after_mm_s = impl_->output.new_velocity[0];
    return step;
}

}  // namespace detail

struct BaseController::Impl {
    Impl(AuxiliaryConfig aux, BaseConfig base)
        : aux_config(std::move(aux)),
          base_config(std::move(base)),
          auxiliary(aux_config),
          logic(aux_config, base_config) {
        init_status = detail::validateBaseConfig(aux_config, base_config);
        // The base control thread is the plugin's only tick() driver, and the
        // plugin sizes its fixed Ruckig shaping period from its own
        // control_rate_hz. Keep that rate slaved to the base rate so a config
        // that only raises base_config.control_rate_hz (e.g. 50 -> 500 Hz)
        // cannot leave the shaper stepping 20 ms of profile per 2 ms tick.
        base_config.chassis.control_rate_hz = base_config.control_rate_hz;
        if (init_status && aux_config.enable_chassis) {
            // The plugin never touches CAN directly: bus ownership stays with
            // the auxiliary controller behind these adapters.
            ChassisWheelInterface io;
            io.send_wheel_velocities =
                [this](const std::array<double, 4>& velocities) {
                    return auxiliary.setChassisWheelVelocities(velocities);
                };
            io.stop_wheels = [this] { return auxiliary.stopChassis(); };
            io.measured_wheel_velocities =
                [this](std::array<double, 4>& velocities, double& age_ms) {
                    if (!auxiliary.refreshChassisFeedback()) return false;
                    const AuxiliaryState snapshot = auxiliary.state();
                    if (!snapshot.chassis_velocity_feedback_valid) return false;
                    velocities = snapshot.chassis_velocities;
                    age_ms = snapshot.chassis_velocity_feedback_max_age_ms;
                    return true;
                };
            chassis = makeChassisController(base_config.chassis_controller,
                                            base_config.chassis,
                                            std::move(io));
            if (!chassis)
                init_status = {ErrorCode::InvalidConfig,
                               "unknown chassis controller type '" +
                                   base_config.chassis_controller + "'"};
        }
    }

    void controlLoop() {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / base_config.control_rate_hz));
        // Absolute deadlines: a slow cycle shortens the next sleep instead of
        // shifting every subsequent cycle (no drift).
        auto next_deadline = Clock::now() + period;
        auto last_cycle = Clock::now();
        while (running.load(std::memory_order_acquire)) {
            std::this_thread::sleep_until(next_deadline);
            next_deadline += period;
            const auto now = Clock::now();
            const double dt_s =
                std::chrono::duration<double>(now - last_cycle).count();
            last_cycle = now;

            std::lock_guard<std::mutex> lock(mutex);
            if (!running.load(std::memory_order_acquire)) break;
            detail::BaseLiftFeedback lift_feedback;
            if (logic.liftDue()) {
                // Blocking SDO read; only performed on decimated lift slots.
                (void)auxiliary.refreshLiftFeedback();
                const AuxiliaryState snapshot = auxiliary.state();
                lift_feedback.position_valid = snapshot.lift_position_valid;
                lift_feedback.position_mm = snapshot.lift_position_mm;
                lift_feedback.velocity_mm_s = snapshot.lift_velocity_mm_s;
                lift_feedback.target_reached = snapshot.lift_target_reached;
            }
            const detail::BaseCycleDecision decision =
                logic.onCycle(lift_feedback);
            if (decision.push_chassis_target && chassis)
                recordFault(chassis->setTarget(decision.chassis_target));
            if (decision.tick_chassis && chassis)
                recordFault(chassis->tick(dt_s));
            if (decision.lift.send)
                recordFault(auxiliary.setLiftPositionMm(
                    decision.lift.position_mm,
                    decision.lift.profile_velocity_mm_s,
                    /*calibration_jog=*/false));
        }
    }

    // Caller holds `mutex`. Transient hardware errors are surfaced through
    // state().fault_reason rather than killing the control thread.
    void recordFault(const Status& status) {
        if (status) return;
        loop_fault = status.message();
    }

    Status startThread() {
        if (running.load(std::memory_order_acquire)) return Status::ok();
        running.store(true, std::memory_order_release);
        control_thread = std::thread([this] { controlLoop(); });
        return Status::ok();
    }

    void stopThread() {
        running.store(false, std::memory_order_release);
        if (control_thread.joinable()) control_thread.join();
    }

    AuxiliaryConfig aux_config;
    BaseConfig base_config;
    Status init_status;
    detail::AuxiliaryController auxiliary;
    std::unique_ptr<IChassisController> chassis;
    detail::BaseCycleLogic logic;

    mutable std::mutex mutex;
    std::thread control_thread;
    std::atomic<bool> running{false};
    bool connected{false};
    bool enabled{false};
    std::string loop_fault;
};

BaseController::BaseController(AuxiliaryConfig auxiliary, BaseConfig base)
    : impl_(std::make_unique<Impl>(std::move(auxiliary), std::move(base))) {}

BaseController::~BaseController() { close(); }

Status BaseController::connect() {
    if (!impl_->init_status) return impl_->init_status;
    if (impl_->connected)
        return {ErrorCode::InvalidState, "base already connected"};
    if (auto status = impl_->auxiliary.connect(); !status) return status;
    impl_->connected = true;
    return Status::ok();
}

Status BaseController::enable() {
    if (!impl_->init_status) return impl_->init_status;
    if (!impl_->connected)
        return {ErrorCode::InvalidState, "base enable requires connect"};
    if (impl_->enabled)
        return {ErrorCode::InvalidState, "base already enabled"};
    if (auto status = impl_->auxiliary.enable(); !status) return status;
    if (impl_->aux_config.enable_chassis) {
        if (auto status = impl_->auxiliary.setChassisEnabled(true); !status)
            return status;
        if (auto status = impl_->chassis->enable(); !status) {
            (void)impl_->auxiliary.setChassisEnabled(false);
            return status;
        }
    }
    if (impl_->aux_config.enable_lift) {
        if (auto status = impl_->auxiliary.setLiftEnabled(true); !status) {
            if (impl_->chassis) (void)impl_->chassis->disable();
            (void)impl_->auxiliary.setChassisEnabled(false);
            return status;
        }
        // Populate the lift snapshot so the first control cycle can prime the
        // z shaping from a measured position.
        (void)impl_->auxiliary.refreshLiftFeedback();
    }
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->logic.clearTarget();
        impl_->loop_fault.clear();
    }
    impl_->enabled = true;
    return impl_->startThread();
}

Status BaseController::disable() {
    if (!impl_->enabled)
        return {ErrorCode::InvalidState, "base is not enabled"};
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->logic.clearTarget();
    }
    // Stop commanding before powering down: join the thread first so no cycle
    // races the teardown below.
    impl_->stopThread();
    Status first_error = Status::ok();
    auto record = [&first_error](const Status& status) {
        if (!status && first_error) first_error = status;
    };
    if (impl_->chassis) {
        record(impl_->chassis->stop());
        record(impl_->chassis->disable());
    }
    if (impl_->aux_config.enable_lift) {
        record(impl_->auxiliary.stopLift());
        record(impl_->auxiliary.setLiftEnabled(false));
    }
    if (impl_->aux_config.enable_chassis)
        record(impl_->auxiliary.setChassisEnabled(false));
    record(impl_->auxiliary.disable());
    impl_->enabled = false;
    return first_error;
}

void BaseController::close() {
    if (!impl_) return;
    if (impl_->enabled) (void)disable();
    impl_->stopThread();
    impl_->auxiliary.close();
    impl_->connected = false;
}

Status BaseController::setTarget(const BaseTarget& target) {
    if (!impl_->enabled)
        return {ErrorCode::InvalidState, "base target requires enable"};
    if (!std::isfinite(target.x_m) || !std::isfinite(target.y_m) ||
        !std::isfinite(target.z_m) || !std::isfinite(target.yaw_rad))
        return {ErrorCode::InvalidArgument, "base target must be finite"};
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->logic.setTarget(target);
    return Status::ok();
}

Status BaseController::stop() {
    if (!impl_->enabled)
        return {ErrorCode::InvalidState, "base stop requires enable"};
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->logic.clearTarget();
    Status first_error = Status::ok();
    if (impl_->chassis) {
        if (auto status = impl_->chassis->stop(); !status && first_error)
            first_error = status;
    }
    if (impl_->aux_config.enable_lift) {
        if (auto status = impl_->auxiliary.stopLift(); !status && first_error)
            first_error = status;
    }
    return first_error;
}

Status BaseController::setCurrentAsOrigin() {
    if (!impl_->enabled)
        return {ErrorCode::InvalidState, "base origin reset requires enable"};
    if (!impl_->chassis) return Status::ok();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->chassis->setCurrentAsOrigin();
}

detail::AuxiliaryController& BaseController::auxiliary() {
    return impl_->auxiliary;
}

const detail::AuxiliaryController& BaseController::auxiliary() const {
    return impl_->auxiliary;
}

BaseState BaseController::state() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    ChassisControllerState chassis_state;
    if (impl_->chassis) chassis_state = impl_->chassis->state();
    std::string extra_fault = impl_->loop_fault;
    const std::string& shaping_fault = impl_->logic.shapingFault();
    if (!shaping_fault.empty()) {
        if (!extra_fault.empty()) extra_fault += "; ";
        extra_fault += shaping_fault;
    }
    return detail::composeBaseState(
        impl_->aux_config.enable_chassis, chassis_state,
        impl_->aux_config.enable_lift, impl_->auxiliary.state(),
        impl_->logic.hasTarget(), impl_->logic.target(), extra_fault);
}

}  // namespace poke
