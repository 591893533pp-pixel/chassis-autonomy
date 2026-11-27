#include "auxiliary.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

#include "elmue_can/canopen_protocol.hpp"
#include "elmue_can/robstride_elmue.h"
#include "elmue_can/zeroerr_canopen.h"

namespace poke::detail {
namespace {

using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;
constexpr int32_t kLiftWrongDirectionGuardCounts = 100;
constexpr int32_t kLiftOvershootGuardCounts = 500;
constexpr int32_t kLiftSoftLimitActiveMarginCounts = 10;
constexpr std::array<uint8_t, 4> kChassisMotorIds{{1, 2, 3, 4}};
constexpr uint8_t kChassisHostId = 0x1F;
constexpr uint16_t kChassisVelocityTargetParameter = 0x700A;
constexpr uint16_t kChassisMechanicalPositionParameter = 0x7019;
constexpr double kChassisHalfTurnRad = kPi;
constexpr double kChassisWrongDirectionGuardRad = 0.15;
constexpr double kChassisOvershootGuardRad = 0.5;
constexpr uint16_t kLiftVendorControlModeObject = 0x2002;
constexpr uint8_t kLiftVendorControlModeSub = 0x01;
constexpr uint16_t kLiftVendorHomeObject = 0x2005;
constexpr uint8_t kLiftVendorHomeSelectionSub = 0x1F;
constexpr uint16_t kLiftVendorMonitorObject = 0x200B;
constexpr uint8_t kLiftVendorMultiturnPositionSub = 0x08;
constexpr uint8_t kLiftVendorEncoderTurnsSub = 0x47;
constexpr uint16_t kLiftVendorPersistObject = 0x200C;
constexpr uint8_t kLiftVendorPersistGateSub = 0x0E;
constexpr uint16_t kLiftCanopenMode = 8;
constexpr uint16_t kLiftTemporaryPositionMode = 1;
constexpr uint16_t kLiftSetCurrentAsHome = 6;
constexpr uint32_t kCanopenStoreSave = 0x65766173;
constexpr int32_t kLiftZeroPositionToleranceCounts = 2;
constexpr int32_t kLiftZeroStationaryJitterCounts = 2;
constexpr int32_t kLiftZeroStationaryVelocityCountsS = 20;
constexpr int kLiftZeroStationarySamples = 6;

}  // namespace

Status validateAuxiliaryConfig(const AuxiliaryConfig& config) {
    if (!config.enable_lift && !config.enable_chassis)
        return {ErrorCode::InvalidConfig, "auxiliary config enables no device"};
    if (config.serial.empty())
        return {ErrorCode::InvalidConfig, "auxiliary CAN serial is empty"};
    if (config.lift_node_id == 0 || config.lift_node_id > 127)
        return {ErrorCode::InvalidConfig, "lift node id must be in [1, 127]"};
    if (!(config.lift_speed_counts_s > 0.0 && config.lift_speed_counts_s <= 1000.0))
        return {ErrorCode::InvalidConfig, "lift speed must be in (0, 1000] counts/s"};
    if (config.lift_travel_counts != 0 &&
        (config.lift_travel_counts < 100 || config.lift_travel_counts > 5000))
        return {ErrorCode::InvalidConfig,
                "lift travel must be 0 (disabled) or in [100, 5000] counts"};
    if (config.lift_calibration_mode &&
        config.lift_control_mode != LiftControlMode::MetricProfilePosition)
        return {ErrorCode::InvalidConfig,
                "lift calibration mode requires metric Profile Position"};
    if (config.enable_lift &&
        config.lift_control_mode == LiftControlMode::MetricProfilePosition) {
        const double counts_per_mm = config.liftCountsPerMm();
        if (!std::isfinite(config.lift_motor_counts_per_revolution) ||
            config.lift_motor_counts_per_revolution <= 0.0 ||
            !std::isfinite(config.lift_reduction_ratio) ||
            config.lift_reduction_ratio <= 0.0 ||
            !std::isfinite(config.lift_screw_lead_mm_per_revolution) ||
            config.lift_screw_lead_mm_per_revolution <= 0.0 ||
            !std::isfinite(counts_per_mm) || counts_per_mm <= 0.0)
            return {ErrorCode::InvalidConfig,
                    "metric lift motor counts, reduction ratio, and screw lead must be positive"};
        if (!std::isfinite(config.lift_min_position_mm) ||
            !std::isfinite(config.lift_max_position_mm) ||
            config.lift_min_position_mm < 0.0 ||
            config.lift_max_position_mm <= config.lift_min_position_mm ||
            config.lift_max_position_mm > kMaximumLiftPositionMm + 1e-9)
            return {ErrorCode::InvalidConfig,
                    "metric lift position range must stay inside 0-250 mm"};
        if (!std::isfinite(config.lift_max_profile_velocity_mm_s) ||
            config.lift_max_profile_velocity_mm_s <= 0.0 ||
            config.lift_max_profile_velocity_mm_s > 200.0 + 1e-9)
            return {ErrorCode::InvalidConfig,
                    "metric lift profile velocity must be within (0, 200] mm/s"};
        if (!std::isfinite(config.lift_position_tolerance_mm) ||
            config.lift_position_tolerance_mm <= 0.0 ||
            config.lift_position_tolerance_mm > 5.0)
            return {ErrorCode::InvalidConfig,
                    "metric lift position tolerance must be in (0, 5] mm"};
        if (config.lift_max_position_mm * counts_per_mm >
            static_cast<double>(std::numeric_limits<int32_t>::max()))
            return {ErrorCode::InvalidConfig,
                    "metric lift maximum position exceeds the signed 32-bit count range"};
        if (config.lift_travel_counts != 0)
            return {ErrorCode::InvalidConfig,
                    "metric lift uses absolute millimetre bounds; lift_travel_counts must be zero"};
        if (config.lift_calibration_mode &&
            (!std::isfinite(config.lift_calibration_max_profile_velocity_mm_s) ||
             config.lift_calibration_max_profile_velocity_mm_s <= 0.0 ||
             config.lift_calibration_max_profile_velocity_mm_s > 5.0 ||
             config.lift_calibration_max_profile_velocity_mm_s >
                 config.lift_max_profile_velocity_mm_s))
            return {ErrorCode::InvalidConfig,
                    "lift calibration speed must be within (0, 5] mm/s and below the metric cap"};
        if (config.lift_calibration_mode &&
            (!std::isfinite(config.lift_calibration_jog_span_mm) ||
             config.lift_calibration_jog_span_mm < 1.0 ||
             config.lift_calibration_jog_span_mm >
                 config.lift_max_position_mm - config.lift_min_position_mm))
            return {ErrorCode::InvalidConfig,
                    "lift calibration jog span must be within [1, configured travel] mm"};
    }
    if (!(config.chassis_speed_rad_s > 0.0 &&
          config.chassis_speed_rad_s <= kMaximumChassisWheelSpeedRadS))
        return {ErrorCode::InvalidConfig, "chassis speed must be in (0, " +
                std::to_string(kMaximumChassisWheelSpeedRadS) + "] rad/s"};
    if (config.chassis_feedback_timeout_ms < 30 ||
        config.chassis_feedback_timeout_ms > 500)
        return {ErrorCode::InvalidConfig,
                "chassis feedback timeout must be in [30, 500] ms"};
    return Status::ok();
}

struct AuxiliaryController::Impl {
    explicit Impl(AuxiliaryConfig value)
        : config(std::move(value)), chassis(kChassisHostId, false), lift(0xFD) {
        elmue_can::canopen::ScaleConfig scale;
        // ScaleConfig is angular by name, but its ratio is generic. Setting
        // counts_per_rev to counts/mm * 2pi makes its numeric SI values mm.
        const double metric_counts_per_mm = config.liftCountsPerMm();
        scale.counts_per_rev =
            config.lift_control_mode == LiftControlMode::MetricProfilePosition &&
                    std::isfinite(metric_counts_per_mm) && metric_counts_per_mm > 0.0
                ? metric_counts_per_mm * 2.0 * kPi
                : 2.0 * kPi;
        scale.sign = 1.0;
        scale.offset_rad = 0.0;
        lift.addNode(config.lift_node_id, scale);
    }

    AuxiliaryConfig config;
    elmue_can::RobstrideElmueDriver chassis;
    elmue_can::ZeroErrCanopenDriver lift;
    std::optional<int32_t> origin_counts;
    // Cached only while io_mutex owns one enabled PP session. Repeated Web
    // waypoints normally keep the same speed, so avoid one blocking 0x6081
    // SDO transaction per setpoint while preserving the public snapshot.
    std::optional<double> programmed_lift_profile_velocity_mm_s;
    std::atomic<bool> stop_requested{false};
    std::atomic<bool> motion_running{false};
    std::thread chassis_worker;
    std::thread lift_worker;
    // The Elmue transport is shared by RobStride and the optional CANopen
    // listener. Serialize lifecycle/parameter transactions issued by public
    // auxiliary APIs; the backend also pauses its stream during transitions.
    std::mutex io_mutex;
    mutable std::mutex mutex;
    AuxiliaryState snapshot;
    std::string chassis_error;
    std::string lift_error;
    bool opened{false};
    bool chassis_ready{false};
    bool lift_attached{false};
    bool lift_ready{false};

    bool metricLift() const {
        return config.lift_control_mode == LiftControlMode::MetricProfilePosition;
    }

    double countsPerMm() const { return config.liftCountsPerMm(); }

    double countsToMm(int32_t counts) const {
        return static_cast<double>(counts) / countsPerMm();
    }

    int32_t mmToCounts(double mm) const {
        return static_cast<int32_t>(std::lround(mm * countsPerMm()));
    }

    bool stopLiftDevice() {
        return metricLift()
            ? lift.haltPositionPP(config.lift_node_id)
            : lift.setTargetVelocity(config.lift_node_id, 0.0);
    }

    void setFault(const std::string& reason) {
        std::lock_guard<std::mutex> lock(mutex);
        if (snapshot.fault_reason.empty()) snapshot.fault_reason = reason;
    }

    std::optional<int32_t> readLiftPositionCounts(double timeout_s = 0.3) {
        const auto result = lift.sdoRead(
            config.lift_node_id, elmue_can::canopen::OD_POSITION_ACTUAL, 0, timeout_s);
        if (!result.success) return std::nullopt;
        const auto value = static_cast<int32_t>(result.value);
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.lift_position_counts = value;
        snapshot.lift_position_valid = true;
        if (metricLift()) snapshot.lift_position_mm = countsToMm(value);
        if (snapshot.lift_limits_valid) {
            snapshot.lift_lower_limit_active =
                value <= snapshot.lift_minimum_counts + kLiftSoftLimitActiveMarginCounts;
            snapshot.lift_upper_limit_active =
                value >= snapshot.lift_maximum_counts - kLiftSoftLimitActiveMarginCounts;
        }
        return value;
    }

    std::optional<double> readLiftVelocityCounts(double timeout_s = 0.3) {
        const auto result = lift.sdoRead(
            config.lift_node_id, elmue_can::canopen::OD_VELOCITY_ACTUAL, 0, timeout_s);
        if (!result.success) return std::nullopt;
        const double value = static_cast<double>(static_cast<int32_t>(result.value));
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.lift_velocity_counts_s = value;
        snapshot.lift_velocity_valid = true;
        if (metricLift()) snapshot.lift_velocity_mm_s = value / countsPerMm();
        return value;
    }

    bool refreshLiftMotionFeedback(double timeout_s = 0.03) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot.lift_position_valid = false;
            snapshot.lift_velocity_valid = false;
        }
        const auto position = readLiftPositionCounts(timeout_s);
        const auto velocity = readLiftVelocityCounts(timeout_s);
        std::optional<uint16_t> statusword;
        if (metricLift()) statusword = lift.readStatusword(config.lift_node_id, timeout_s);
        const auto cached = lift.getFeedback(config.lift_node_id);
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (statusword) snapshot.lift_statusword = *statusword;
            else if (cached.online) snapshot.lift_statusword = cached.statusword;
            if (cached.online) snapshot.lift_error_code = cached.error_code;
            if (metricLift() && position && velocity && snapshot.lift_target_valid &&
                !snapshot.lift_halted) {
                const bool position_close =
                    std::abs(snapshot.lift_position_mm -
                             snapshot.lift_target_position_mm) <=
                    config.lift_position_tolerance_mm;
                const bool drive_reached =
                    (snapshot.lift_statusword &
                     elmue_can::canopen::SW_BIT_TARGET_REACHED) != 0;
                const bool settled = std::abs(snapshot.lift_velocity_mm_s) <= 0.5;
                snapshot.lift_target_reached = position_close &&
                                               (drive_reached || settled);
                if (snapshot.lift_target_reached) {
                    snapshot.lift_halted = false;
                    const bool chassis_moving = std::any_of(
                        snapshot.chassis_commanded_velocities.begin(),
                        snapshot.chassis_commanded_velocities.end(),
                        [](double value) { return std::abs(value) > 1e-9; });
                    snapshot.motion_running = chassis_moving;
                }
            }
        }
        return position.has_value() && velocity.has_value() &&
               (!metricLift() || statusword.has_value());
    }

    std::optional<std::array<double, 4>> readChassisPositions() {
        std::array<double, 4> positions{};
        for (std::size_t i = 0; i < kChassisMotorIds.size(); ++i) {
            const auto result = chassis.readParameter(
                kChassisMotorIds[i], kChassisMechanicalPositionParameter, 0.2);
            if (!result || !result->success || !std::isfinite(result->value))
                return std::nullopt;
            positions[i] = result->value;
        }
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.chassis_positions = positions;
        snapshot.chassis_positions_valid = true;
        return positions;
    }

    Status captureChassisVelocityFeedback() {
        std::array<double, 4> velocities{};
        std::array<double, 4> temperatures{};
        std::array<uint8_t, 4> fault_codes{};
        const auto now = Clock::now();
        double max_age_ms = 0.0;
        bool all_valid = true;
        uint8_t missing_id = 0;
        uint8_t faulted_id = 0;
        for (std::size_t i = 0; i < kChassisMotorIds.size(); ++i) {
            const auto feedback = chassis.getMotorFeedback(kChassisMotorIds[i]);
            const double age_ms = feedback.is_valid
                ? std::max(0.0, std::chrono::duration<double, std::milli>(
                      now - feedback.last_update).count())
                : static_cast<double>(config.chassis_feedback_timeout_ms) + 1.0;
            max_age_ms = std::max(max_age_ms, age_ms);
            const bool valid = feedback.is_valid &&
                std::isfinite(feedback.velocity) &&
                std::isfinite(feedback.temperature) &&
                age_ms <= static_cast<double>(config.chassis_feedback_timeout_ms);
            if (!valid) {
                all_valid = false;
                if (missing_id == 0) missing_id = kChassisMotorIds[i];
                continue;
            }
            velocities[i] = feedback.velocity;
            temperatures[i] = feedback.temperature;
            fault_codes[i] = feedback.fault_code;
            if (feedback.fault_code != 0 && faulted_id == 0)
                faulted_id = kChassisMotorIds[i];
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot.chassis_velocities = velocities;
            snapshot.chassis_temperatures = temperatures;
            snapshot.chassis_fault_codes = fault_codes;
            snapshot.chassis_velocity_feedback_max_age_ms = max_age_ms;
            snapshot.chassis_velocity_feedback_valid = all_valid && faulted_id == 0;
        }
        if (faulted_id != 0)
            return {ErrorCode::MotorFault,
                    "chassis motor " + std::to_string(faulted_id) +
                        " active report contains a fault"};
        if (!all_valid)
            return {ErrorCode::FeedbackStale,
                    "missing or stale active velocity report from chassis motor " +
                        std::to_string(missing_id) + " (max age_ms=" +
                        std::to_string(max_age_ms) + ", limit_ms=" +
                        std::to_string(config.chassis_feedback_timeout_ms) + ")"};
        return Status::ok();
    }

    bool sendWheelVelocities(const std::array<double, 4>& velocities) {
        bool ok = true;
        for (std::size_t i = 0; i < kChassisMotorIds.size(); ++i)
            ok = chassis.writeParameter(
                     kChassisMotorIds[i], kChassisVelocityTargetParameter,
                     static_cast<float>(velocities[i])) && ok;
        return ok;
    }

    bool sendAllWheelVelocities(double velocity) {
        std::array<double, 4> velocities{};
        velocities.fill(velocity);
        return sendWheelVelocities(velocities);
    }

    Status setupChassis() {
        bool ok = true;
        for (uint8_t id : kChassisMotorIds) {
            chassis.setMotorType(id, elmue_can::MotorType::RS02);
            ok = chassis.disableMotor(id, true) && ok;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            ok = chassis.writeParameter(id, kChassisVelocityTargetParameter, 0.0f) && ok;
            ok = chassis.setRunMode(id, elmue_can::RunMode::VELOCITY) && ok;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            ok = chassis.enableMotor(id) && ok;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            ok = chassis.setActiveReport(id, true) && ok;
        }
        std::optional<std::array<double, 4>> positions;
        for (int attempt = 0; attempt < 3 && !positions; ++attempt) {
            positions = readChassisPositions();
            if (!positions) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        Status feedback_status{ErrorCode::FeedbackStale,
                               "waiting for chassis active velocity reports"};
        for (int attempt = 0; attempt < 20 && !feedback_status; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            feedback_status = captureChassisVelocityFeedback();
            if (feedback_status.code() == ErrorCode::MotorFault) break;
        }
        chassis_ready = ok && positions.has_value() && feedback_status;
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot.chassis_enabled = chassis_ready;
            snapshot.enabled = chassis_ready || lift_ready;
            if (chassis_ready) {
                snapshot.chassis_commanded_velocities.fill(0.0);
                snapshot.stop_requested = false;
            }
        }
        if (!chassis_ready) {
            (void)sendAllWheelVelocities(0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            for (uint8_t id : kChassisMotorIds) {
                (void)chassis.setActiveReport(id, false);
                (void)chassis.disableMotor(id, false);
            }
        }
        if (chassis_ready) return Status::ok();
        if (!feedback_status) return feedback_status;
        return {ErrorCode::TransportError,
                "chassis motors 1-4 velocity setup failed"};
    }

    Status setupLift() {
        programmed_lift_profile_velocity_mm_s.reset();
        {
            std::lock_guard<std::mutex> lock(mutex);
            // A software range belongs only to one physical enable session.
            // Never expose a previous origin while a new setup is in progress
            // or after that setup fails.
            snapshot.lift_position_valid = false;
            snapshot.lift_velocity_valid = false;
            snapshot.lift_limits_valid = false;
            snapshot.lift_lower_limit_active = false;
            snapshot.lift_upper_limit_active = false;
            snapshot.lift_error_code = 0;
        }
        if (!lift_attached) {
            lift.attach(&chassis.elmue());
            lift_attached = true;
            chassis.setForeignStdFrameCallback(
                [this](uint32_t id, const uint8_t* data, uint8_t dlc) {
                    lift.handleRxFrame(id, data, dlc);
                });
        }
        const auto retry = [](const auto& operation) {
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (operation()) return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            return false;
        };
        const bool nmt_ok = retry([&] { return lift.nmtStart(config.lift_node_id); });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const auto requested_mode = metricLift()
            ? elmue_can::canopen::Cia402Mode::PP
            : elmue_can::canopen::Cia402Mode::PV;
        const bool zero_ok = metricLift() || retry([&] {
            return lift.setTargetVelocity(config.lift_node_id, 0.0);
        });
        const bool mode_set_ok = retry([&] {
            return lift.setMode(config.lift_node_id, requested_mode, 0.5);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::optional<elmue_can::canopen::Cia402Mode> mode;
        for (int attempt = 0; attempt < 3 && !mode; ++attempt) {
            mode = lift.readMode(config.lift_node_id, 0.5);
            if (!mode) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        const bool mode_read_ok = mode && *mode == requested_mode;
        origin_counts.reset();
        for (int attempt = 0; attempt < 3 && !origin_counts; ++attempt) {
            origin_counts = readLiftPositionCounts();
            if (!origin_counts) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        const bool enabled = retry([&] { return lift.enable(config.lift_node_id, 2.0); });
        lift_ready = enabled;
        bool metric_position_in_range = true;
        bool initial_hold_ok = true;
        double initial_position_mm = 0.0;
        const double initial_hold_velocity_mm_s =
            std::min(config.lift_max_profile_velocity_mm_s, 1.0);
        if (metricLift() && origin_counts) {
            initial_position_mm = countsToMm(*origin_counts);
            metric_position_in_range =
                initial_position_mm >=
                    config.lift_min_position_mm - config.lift_position_tolerance_mm &&
                initial_position_mm <=
                    config.lift_max_position_mm + config.lift_position_tolerance_mm;
            const bool position_accepted =
                metric_position_in_range || config.lift_calibration_mode;
            initial_hold_ok = enabled && position_accepted && retry([&] {
                return lift.moveToPositionPP(
                    config.lift_node_id, initial_position_mm,
                    initial_hold_velocity_mm_s, true, false);
            });
            if (initial_hold_ok)
                programmed_lift_profile_velocity_mm_s =
                    initial_hold_velocity_mm_s;
        }
        std::optional<uint16_t> statusword;
        if (enabled) statusword = lift.readStatusword(config.lift_node_id, 0.3);
        const auto error = lift.sdoRead(
            config.lift_node_id, elmue_can::canopen::OD_ERROR_CODE, 0, 0.3);
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot.lift_enabled = lift_ready;
            snapshot.enabled = chassis_ready || lift_ready;
            if (origin_counts) {
                if (metricLift()) {
                    snapshot.lift_origin_counts = 0;
                    snapshot.lift_minimum_counts = mmToCounts(config.lift_min_position_mm);
                    snapshot.lift_maximum_counts = mmToCounts(config.lift_max_position_mm);
                    snapshot.lift_limits_valid = true;
                    snapshot.lift_position_mm = initial_position_mm;
                    snapshot.lift_target_position_mm = initial_position_mm;
                    snapshot.lift_profile_velocity_mm_s =
                        initial_hold_velocity_mm_s;
                    snapshot.lift_target_valid = initial_hold_ok;
                    snapshot.lift_target_reached = initial_hold_ok;
                    snapshot.lift_halted = false;
                } else if (config.lift_travel_counts > 0) {
                    snapshot.lift_origin_counts = *origin_counts;
                    snapshot.lift_minimum_counts =
                        *origin_counts - config.lift_travel_counts;
                    snapshot.lift_maximum_counts =
                        *origin_counts + config.lift_travel_counts;
                    snapshot.lift_limits_valid = true;
                } else {
                    snapshot.lift_origin_counts = *origin_counts;
                    snapshot.lift_minimum_counts = 0;
                    snapshot.lift_maximum_counts = 0;
                    snapshot.lift_limits_valid = false;
                }
                snapshot.lift_lower_limit_active =
                    snapshot.lift_limits_valid &&
                    *origin_counts <= snapshot.lift_minimum_counts +
                                          kLiftSoftLimitActiveMarginCounts;
                snapshot.lift_upper_limit_active =
                    snapshot.lift_limits_valid &&
                    *origin_counts >= snapshot.lift_maximum_counts -
                                          kLiftSoftLimitActiveMarginCounts;
            }
            snapshot.lift_commanded_velocity_counts_s = 0.0;
            if (statusword) snapshot.lift_statusword = *statusword;
            if (error.success)
                snapshot.lift_error_code = static_cast<uint16_t>(error.value & 0xFFFF);
        }
        const bool position_accepted =
            metric_position_in_range || config.lift_calibration_mode;
        if (nmt_ok && zero_ok && mode_set_ok && mode_read_ok && origin_counts &&
            enabled && position_accepted && initial_hold_ok && statusword &&
            error.success && error.value == 0)
            return Status::ok();
        std::ostringstream detail;
        detail << "lift " << (metricLift() ? "metric PP" : "raw PV")
               << " setup failed (nmt=" << nmt_ok << ", zero=" << zero_ok
               << ", mode_set=" << mode_set_ok << ", mode_read=" << mode_read_ok
               << ", position=" << origin_counts.has_value() << ", enable=" << enabled
               << ", position_in_range=" << metric_position_in_range
               << ", calibration_mode=" << config.lift_calibration_mode
               << ", initial_hold=" << initial_hold_ok
               << ", status=" << statusword.has_value()
               << ", error=0x" << std::hex << error.value << std::dec << ')';
        if (!lift.lastError().empty()) detail << ": " << lift.lastError();
        return {ErrorCode::TransportError, detail.str()};
    }

    bool moveLiftToward(int32_t target, int direction) {
        const auto segment_start = readLiftPositionCounts();
        if (!segment_start) { lift_error = "unable to read position before segment"; return false; }
        if (!lift.setTargetVelocity(config.lift_node_id,
                                    direction * config.lift_speed_counts_s)) {
            lift_error = "unable to send velocity"; return false;
        }
        const double expected_seconds =
            std::abs(static_cast<double>(target - *segment_start)) /
            config.lift_speed_counts_s;
        const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(expected_seconds + 2.0));
        int read_failures = 0;
        while (!stop_requested.load() && Clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const auto actual = readLiftPositionCounts();
            if (!actual) {
                if (++read_failures >= 3) {
                    lift_error = "three consecutive position read failures"; return false;
                }
                continue;
            }
            read_failures = 0;
            const int32_t traveled = *actual - *segment_start;
            if ((direction > 0 && traveled < -kLiftWrongDirectionGuardCounts) ||
                (direction < 0 && traveled > kLiftWrongDirectionGuardCounts)) {
                lift_error = "moved in the opposite direction"; return false;
            }
            const int64_t allowed =
                std::abs(static_cast<int64_t>(target) - *segment_start) +
                kLiftOvershootGuardCounts;
            if (std::abs(static_cast<int64_t>(traveled)) > allowed) {
                lift_error = "exceeded relative displacement guard"; return false;
            }
            if ((direction > 0 && *actual >= target) ||
                (direction < 0 && *actual <= target)) {
                lift.setTargetVelocity(config.lift_node_id, 0.0); return true;
            }
        }
        lift_error = stop_requested.load() ? "motion interrupted" : "segment timed out";
        return false;
    }

    void runLiftUpDown() {
        const int32_t origin = *origin_counts;
        const int32_t upper = origin + config.lift_travel_counts;
        bool ok = moveLiftToward(upper, +1);
        lift.setTargetVelocity(config.lift_node_id, 0.0);
        if (ok && !stop_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            ok = moveLiftToward(origin, -1);
        }
        lift.setTargetVelocity(config.lift_node_id, 0.0);
        readLiftPositionCounts();
        if (!ok && lift_error.empty()) lift_error = "up/down did not complete";
    }

    bool moveChassisToDelta(const std::array<double, 4>& origin,
                            const std::array<double, 4>& segment_start,
                            double target_delta, int direction) {
        std::array<bool, 4> reached{};
        std::array<double, 4> command{};
        command.fill(direction * config.chassis_speed_rad_s);
        const double max_segment = kChassisHalfTurnRad / config.chassis_speed_rad_s + 3.0;
        const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(max_segment));
        auto next = Clock::now();
        auto next_poll = Clock::now();
        int read_failures = 0;
        int send_failures = 0;
        while (!stop_requested.load() && Clock::now() < deadline) {
            if (!sendWheelVelocities(command)) {
                if (++send_failures >= 3) {
                    chassis_error = "three consecutive velocity send failures"; return false;
                }
            } else send_failures = 0;
            const auto now = Clock::now();
            if (now >= next_poll) {
                next_poll = now + std::chrono::milliseconds(100);
                const auto actual = readChassisPositions();
                if (!actual) {
                    if (++read_failures >= 3) {
                        chassis_error = "three consecutive MECH_POS read failures"; return false;
                    }
                } else {
                    read_failures = 0;
                    bool all_reached = true;
                    for (std::size_t i = 0; i < actual->size(); ++i) {
                        const double delta = (*actual)[i] - origin[i];
                        const double segment_delta = (*actual)[i] - segment_start[i];
                        const bool wrong_direction = direction > 0
                            ? segment_delta < -kChassisWrongDirectionGuardRad
                            : segment_delta > kChassisWrongDirectionGuardRad;
                        if (wrong_direction) {
                            chassis_error = "motor " + std::to_string(kChassisMotorIds[i]) +
                                            " moved in the opposite direction";
                            return false;
                        }
                        if (delta < -kChassisOvershootGuardRad ||
                            delta > kChassisHalfTurnRad + kChassisOvershootGuardRad) {
                            chassis_error = "motor " + std::to_string(kChassisMotorIds[i]) +
                                            " exceeded half-turn displacement guard";
                            return false;
                        }
                        if (!reached[i] && ((direction > 0 && delta >= target_delta) ||
                                            (direction < 0 && delta <= target_delta))) {
                            reached[i] = true; command[i] = 0.0;
                        }
                        all_reached = all_reached && reached[i];
                    }
                    if (all_reached) { sendAllWheelVelocities(0.0); return true; }
                }
            }
            next += std::chrono::milliseconds(10);
            std::this_thread::sleep_until(next);
        }
        chassis_error = stop_requested.load() ? "motion interrupted"
                                               : "half-turn segment timed out";
        return false;
    }

    void runChassisHalfTurn() {
        const auto origin = readChassisPositions();
        if (!origin) {
            chassis_error = "unable to read starting MECH_POS";
            sendAllWheelVelocities(0.0); return;
        }
        bool ok = moveChassisToDelta(*origin, *origin, kChassisHalfTurnRad, +1);
        sendAllWheelVelocities(0.0);
        if (ok && !stop_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            const auto turn_position = readChassisPositions();
            if (!turn_position) {
                chassis_error = "unable to read half-turn MECH_POS"; ok = false;
            } else ok = moveChassisToDelta(*origin, *turn_position, 0.0, -1);
        }
        sendAllWheelVelocities(0.0);
        readChassisPositions();
        if (!ok && chassis_error.empty()) chassis_error = "half-turn out/back did not complete";
    }

    void updateMotionState() {
        const bool running = chassis_worker.joinable() || lift_worker.joinable();
        motion_running.store(running);
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.motion_running = running;
        snapshot.stop_requested = stop_requested.load();
    }
};

AuxiliaryController::AuxiliaryController(AuxiliaryConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    impl_->snapshot.configured = true;
    impl_->snapshot.chassis_configured = impl_->config.enable_chassis;
    impl_->snapshot.lift_configured = impl_->config.enable_lift;
    impl_->snapshot.lift_metric_position_configured = impl_->metricLift();
    if (impl_->metricLift()) {
        impl_->snapshot.lift_counts_per_mm = impl_->countsPerMm();
        impl_->snapshot.lift_min_position_mm = impl_->config.lift_min_position_mm;
        impl_->snapshot.lift_max_position_mm = impl_->config.lift_max_position_mm;
    }
}

AuxiliaryController::~AuxiliaryController() { close(); }

Status AuxiliaryController::connect() {
    if (auto status = validateAuxiliaryConfig(impl_->config); !status) return status;
    if (impl_->opened) return {ErrorCode::InvalidState, "auxiliary bus already connected"};
    if (!impl_->chassis.init(impl_->config.serial, 1'000'000, 0))
        return {ErrorCode::TransportError,
                "auxiliary CAN open failed: " + impl_->chassis.lastError()};
    impl_->opened = true;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->snapshot.connected = true;
    return Status::ok();
}

Status AuxiliaryController::enable() {
    if (!impl_->opened) return {ErrorCode::InvalidState, "auxiliary enable requires connect"};
    if (!impl_->config.enable_on_robot_enable) return Status::ok();
    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    Status status = Status::ok();
    if (impl_->config.enable_chassis) status = impl_->setupChassis();
    if (status && impl_->config.enable_lift) status = impl_->setupLift();
    if (!status) { impl_->setFault(status.message()); disable(); return status; }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->snapshot.enabled = impl_->chassis_ready || impl_->lift_ready;
    return Status::ok();
}

Status AuxiliaryController::startMotion() {
    if ((!impl_->chassis_ready && impl_->config.enable_chassis) ||
        (!impl_->lift_ready && impl_->config.enable_lift))
        return {ErrorCode::InvalidState, "auxiliary motion requires enabled devices"};
    if (impl_->chassis_worker.joinable() || impl_->lift_worker.joinable())
        return {ErrorCode::Busy, "auxiliary motion already active"};
    if (impl_->config.enable_lift && impl_->config.lift_travel_counts == 0)
        return {ErrorCode::InvalidConfig,
                "fixed lift up/down motion requires a nonzero lift travel"};
    impl_->stop_requested.store(false);
    impl_->chassis_error.clear();
    impl_->lift_error.clear();
    { std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->snapshot.fault_reason.clear(); }
    if (impl_->config.enable_lift) {
        impl_->origin_counts = impl_->readLiftPositionCounts();
        if (!impl_->origin_counts)
            return {ErrorCode::FeedbackStale, "unable to refresh lift position before motion"};
        impl_->lift_worker = std::thread([this] { impl_->runLiftUpDown(); });
    }
    if (impl_->config.enable_chassis)
        impl_->chassis_worker = std::thread([this] { impl_->runChassisHalfTurn(); });
    impl_->updateMotionState();
    return Status::ok();
}

void AuxiliaryController::requestStop() {
    impl_->stop_requested.store(true);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->snapshot.stop_requested = true;
}

Status AuxiliaryController::waitMotion() {
    if (impl_->chassis_worker.joinable()) impl_->chassis_worker.join();
    if (impl_->lift_worker.joinable()) impl_->lift_worker.join();
    if (impl_->chassis_ready) impl_->sendAllWheelVelocities(0.0);
    if (impl_->lift_attached && impl_->lift_ready) (void)impl_->stopLiftDevice();
    impl_->updateMotionState();
    std::string error;
    if (!impl_->chassis_error.empty()) error = "chassis: " + impl_->chassis_error;
    if (!impl_->lift_error.empty()) {
        if (!error.empty()) error += "; ";
        error += "lift: " + impl_->lift_error;
    }
    if (!error.empty()) {
        impl_->setFault(error);
        return {impl_->stop_requested.load() ? ErrorCode::Cancelled : ErrorCode::TransportError,
                error};
    }
    return Status::ok();
}

Status AuxiliaryController::stopMotion() {
    requestStop();
    return waitMotion();
}

Status AuxiliaryController::setChassisEnabled(bool enabled) {
    if (!impl_->config.enable_chassis)
        return {ErrorCode::InvalidState, "chassis hardware is not configured"};
    if (!impl_->opened)
        return {ErrorCode::InvalidState, "chassis lifecycle requires auxiliary connect"};
    if (impl_->chassis_worker.joinable())
        return {ErrorCode::Busy, "fixed auxiliary chassis motion is active"};

    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    if (enabled) {
        if (impl_->chassis_ready) return Status::ok();
        impl_->stop_requested.store(false);
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->snapshot.stop_requested = false;
            impl_->snapshot.fault_reason.clear();
        }
        const auto status = impl_->setupChassis();
        if (!status) impl_->setFault(status.message());
        return status;
    }

    bool ok = true;
    if (impl_->chassis_ready) {
        ok = impl_->sendAllWheelVelocities(0.0) && ok;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->snapshot.chassis_positions_valid = false;
        }
        // The velocity stream does not block on SDO-style parameter reads.
        // Refresh the UI-facing MECH_POS snapshot only after zero velocity has
        // settled, immediately before the drives are disabled.
        (void)impl_->readChassisPositions();
        for (uint8_t id : kChassisMotorIds) {
            ok = impl_->chassis.setActiveReport(id, false) && ok;
            ok = impl_->chassis.disableMotor(id, false) && ok;
        }
    }
    impl_->chassis_ready = false;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.chassis_enabled = false;
        impl_->snapshot.enabled = impl_->lift_ready;
        impl_->snapshot.chassis_velocity_feedback_valid = false;
        impl_->snapshot.motion_running =
            std::abs(impl_->snapshot.lift_commanded_velocity_counts_s) > 1e-9;
        impl_->snapshot.chassis_commanded_velocities.fill(0.0);
    }
    return ok ? Status::ok()
              : Status{ErrorCode::TransportError,
                       "one or more chassis motors failed to stop/disable"};
}

Status AuxiliaryController::setChassisWheelVelocities(
    const std::array<double, 4>& velocities_rad_s) {
    if (!impl_->config.enable_chassis)
        return {ErrorCode::InvalidState, "chassis hardware is not configured"};
    if (!impl_->opened || !impl_->chassis_ready)
        return {ErrorCode::InvalidState, "chassis velocity requires enabled motors"};
    for (std::size_t index = 0; index < velocities_rad_s.size(); ++index) {
        const double value = velocities_rad_s[index];
        if (!std::isfinite(value))
            return {ErrorCode::InvalidArgument,
                    "chassis wheel velocity must be finite"};
        if (std::abs(value) > impl_->config.chassis_speed_rad_s + 1e-9)
            return {ErrorCode::LimitViolation,
                    "chassis wheel velocity exceeds configured limit"};
    }
    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    if (!impl_->sendWheelVelocities(velocities_rad_s))
        return {ErrorCode::TransportError, "chassis velocity write failed"};
    const bool moving = std::any_of(
        velocities_rad_s.begin(), velocities_rad_s.end(),
        [](double value) { return std::abs(value) > 1e-9; });
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.chassis_commanded_velocities = velocities_rad_s;
        impl_->snapshot.motion_running = moving ||
            std::abs(impl_->snapshot.lift_commanded_velocity_counts_s) > 1e-9;
        impl_->snapshot.stop_requested = !moving;
    }
    return Status::ok();
}

Status AuxiliaryController::stopChassis() {
    if (!impl_->config.enable_chassis || !impl_->opened || !impl_->chassis_ready) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.chassis_commanded_velocities.fill(0.0);
        impl_->snapshot.motion_running = false;
        return Status::ok();
    }
    std::array<double, 4> stopped{};
    const auto status = setChassisWheelVelocities(stopped);
    if (!status) return status;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.chassis_positions_valid = false;
    }
    // Position refresh is diagnostic only: a failed read must not turn a
    // successfully transmitted zero target into a failed safety stop.
    (void)impl_->readChassisPositions();
    return Status::ok();
}

Status AuxiliaryController::refreshChassisFeedback() {
    if (!impl_->config.enable_chassis || !impl_->opened || !impl_->chassis_ready)
        return {ErrorCode::InvalidState,
                "chassis feedback refresh requires enabled motors"};
    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    return impl_->captureChassisVelocityFeedback();
}

Status AuxiliaryController::setLiftEnabled(bool enabled) {
    if (!impl_->config.enable_lift)
        return {ErrorCode::InvalidState, "lift hardware is not configured"};
    if (!impl_->opened)
        return {ErrorCode::InvalidState, "lift lifecycle requires auxiliary connect"};
    if (impl_->lift_worker.joinable())
        return {ErrorCode::Busy, "fixed auxiliary lift motion is active"};

    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    if (enabled) {
        if (impl_->lift_ready) return Status::ok();
        impl_->stop_requested.store(false);
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->snapshot.stop_requested = false;
            impl_->snapshot.fault_reason.clear();
        }
        const auto status = impl_->setupLift();
        if (!status) {
            impl_->setFault(status.message());
            if (impl_->lift_ready) (void)impl_->stopLiftDevice();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            (void)impl_->lift.disable(impl_->config.lift_node_id);
            (void)impl_->lift.nmtEnterPreOp(impl_->config.lift_node_id);
            impl_->lift_ready = false;
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->snapshot.lift_enabled = false;
            impl_->snapshot.enabled = impl_->chassis_ready;
        }
        return status;
    }

    bool ok = true;
    if (impl_->lift_attached) {
        if (impl_->lift_ready) ok = impl_->stopLiftDevice() && ok;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        (void)impl_->refreshLiftMotionFeedback(0.05);
        if (impl_->lift_ready)
            ok = impl_->lift.disable(impl_->config.lift_node_id) && ok;
        ok = impl_->lift.nmtEnterPreOp(impl_->config.lift_node_id) && ok;
    }
    impl_->lift_ready = false;
    impl_->programmed_lift_profile_velocity_mm_s.reset();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_enabled = false;
        impl_->snapshot.enabled = impl_->chassis_ready;
        impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        impl_->snapshot.lift_target_valid = false;
        impl_->snapshot.lift_target_reached = false;
        impl_->snapshot.lift_halted = false;
        impl_->snapshot.lift_limits_valid = false;
        impl_->snapshot.lift_lower_limit_active = false;
        impl_->snapshot.lift_upper_limit_active = false;
        const bool chassis_moving = std::any_of(
            impl_->snapshot.chassis_commanded_velocities.begin(),
            impl_->snapshot.chassis_commanded_velocities.end(),
            [](double velocity) { return std::abs(velocity) > 1e-9; });
        impl_->snapshot.motion_running = chassis_moving;
    }
    return ok ? Status::ok()
              : Status{ErrorCode::TransportError,
                       "lift failed to stop/disable or enter NMT pre-operational"};
}

Status AuxiliaryController::setLiftVelocity(double velocity_counts_s) {
    if (!impl_->config.enable_lift)
        return {ErrorCode::InvalidState, "lift hardware is not configured"};
    if (!impl_->opened || !impl_->lift_ready)
        return {ErrorCode::InvalidState, "lift velocity requires an enabled drive"};
    if (impl_->metricLift())
        return {ErrorCode::InvalidState,
                "raw lift velocity is unavailable in metric Profile Position mode"};
    if (!std::isfinite(velocity_counts_s))
        return {ErrorCode::InvalidArgument, "lift velocity must be finite"};
    if (std::abs(velocity_counts_s) > impl_->config.lift_speed_counts_s + 1e-9)
        return {ErrorCode::LimitViolation,
                "lift velocity exceeds the configured limit"};

    if (std::abs(velocity_counts_s) > 1e-9) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->snapshot.lift_position_valid)
            return {ErrorCode::FeedbackStale,
                    "lift velocity requires fresh position feedback"};
        if (impl_->snapshot.lift_limits_valid &&
            ((velocity_counts_s > 0.0 && impl_->snapshot.lift_upper_limit_active) ||
             (velocity_counts_s < 0.0 && impl_->snapshot.lift_lower_limit_active)))
            return {ErrorCode::LimitViolation,
                    "lift command points outward at the active software limit"};
    }

    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    if (!impl_->lift.setTargetVelocity(impl_->config.lift_node_id,
                                       velocity_counts_s))
        return {ErrorCode::TransportError, "lift target-velocity write failed"};
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_commanded_velocity_counts_s = velocity_counts_s;
        const bool chassis_moving = std::any_of(
            impl_->snapshot.chassis_commanded_velocities.begin(),
            impl_->snapshot.chassis_commanded_velocities.end(),
            [](double velocity) { return std::abs(velocity) > 1e-9; });
        impl_->snapshot.motion_running = chassis_moving ||
                                         std::abs(velocity_counts_s) > 1e-9;
    }
    return Status::ok();
}

Status AuxiliaryController::setLiftPositionMm(double position_mm,
                                              double profile_velocity_mm_s,
                                              bool calibration_jog) {
    if (!impl_->config.enable_lift)
        return {ErrorCode::InvalidState, "lift hardware is not configured"};
    if (!impl_->opened || !impl_->lift_ready)
        return {ErrorCode::InvalidState,
                "lift position requires an enabled drive"};
    if (!impl_->metricLift())
        return {ErrorCode::InvalidState,
                "metric lift position requires Profile Position configuration"};
    if (!std::isfinite(position_mm) || !std::isfinite(profile_velocity_mm_s))
        return {ErrorCode::InvalidArgument,
                "lift position and profile velocity must be finite"};
    if (calibration_jog && !impl_->config.lift_calibration_mode)
        return {ErrorCode::InvalidState,
                "lift calibration jog requires explicit calibration mode"};
    if (!calibration_jog &&
        (position_mm < impl_->config.lift_min_position_mm - 1e-9 ||
         position_mm > impl_->config.lift_max_position_mm + 1e-9))
        return {ErrorCode::LimitViolation,
                "lift position is outside the configured metric range"};
    const double velocity_limit = calibration_jog
        ? impl_->config.lift_calibration_max_profile_velocity_mm_s
        : impl_->config.lift_max_profile_velocity_mm_s;
    if (profile_velocity_mm_s <= 0.0 ||
        profile_velocity_mm_s > velocity_limit + 1e-9)
        return {ErrorCode::LimitViolation,
                calibration_jog
                    ? "lift calibration jog exceeds the reduced speed cap"
                    : "lift profile velocity is outside the configured metric range"};
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->snapshot.lift_position_valid)
            return {ErrorCode::FeedbackStale,
                    "lift position command requires fresh position feedback"};
        if (calibration_jog &&
            std::abs(position_mm - impl_->snapshot.lift_position_mm) >
                impl_->config.lift_calibration_jog_span_mm + 1e-9)
            return {ErrorCode::LimitViolation,
                    "lift calibration jog target exceeds the relative segment span"};
        if (!calibration_jog &&
            (impl_->snapshot.lift_position_mm <
                impl_->config.lift_min_position_mm -
                    impl_->config.lift_position_tolerance_mm ||
            impl_->snapshot.lift_position_mm >
                impl_->config.lift_max_position_mm +
                    impl_->config.lift_position_tolerance_mm))
            return {ErrorCode::LimitViolation,
                    "current lift position is outside the configured metric range; re-zero at the verified mechanical reference"};
    }
    const double target_counts = position_mm * impl_->countsPerMm();
    if (target_counts < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
        target_counts > static_cast<double>(std::numeric_limits<int32_t>::max()))
        return {ErrorCode::LimitViolation,
                "lift position target exceeds the signed 32-bit drive coordinate"};

    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    const bool profile_velocity_unchanged =
        impl_->programmed_lift_profile_velocity_mm_s.has_value() &&
        std::abs(*impl_->programmed_lift_profile_velocity_mm_s -
                 profile_velocity_mm_s) <= 1e-9;
    // A zero argument tells the CANopen driver to retain its already
    // programmed 0x6081 value while still issuing the target/new-setpoint
    // handshake. On failure, leave the cache unchanged so the next retry
    // conservatively writes the requested profile velocity again.
    const double profile_velocity_write_mm_s =
        profile_velocity_unchanged ? 0.0 : profile_velocity_mm_s;
    if (!impl_->lift.moveToPositionPP(
            impl_->config.lift_node_id, position_mm,
            profile_velocity_write_mm_s, true, false))
        return {ErrorCode::TransportError,
                "lift Profile Position setpoint write failed"};
    impl_->programmed_lift_profile_velocity_mm_s = profile_velocity_mm_s;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_target_position_mm = position_mm;
        impl_->snapshot.lift_profile_velocity_mm_s = profile_velocity_mm_s;
        impl_->snapshot.lift_commanded_velocity_counts_s =
            profile_velocity_mm_s * impl_->countsPerMm();
        impl_->snapshot.lift_target_valid = true;
        impl_->snapshot.lift_target_reached = false;
        impl_->snapshot.lift_halted = false;
        impl_->snapshot.motion_running = true;
    }
    return Status::ok();
}

Status AuxiliaryController::stopLift() {
    if (!impl_->config.enable_lift || !impl_->opened || !impl_->lift_ready) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        return Status::ok();
    }
    if (impl_->metricLift()) {
        std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
        if (!impl_->stopLiftDevice())
            return {ErrorCode::TransportError,
                    "lift Profile Position HALT write failed"};
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        impl_->snapshot.lift_target_reached = false;
        impl_->snapshot.lift_halted = true;
        const bool chassis_moving = std::any_of(
            impl_->snapshot.chassis_commanded_velocities.begin(),
            impl_->snapshot.chassis_commanded_velocities.end(),
            [](double velocity) { return std::abs(velocity) > 1e-9; });
        impl_->snapshot.motion_running = chassis_moving;
    } else {
        const auto status = setLiftVelocity(0.0);
        if (!status) return status;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return refreshLiftFeedback();
}

Status AuxiliaryController::setLiftCurrentPositionAsZero() {
    if (!impl_->config.enable_lift)
        return {ErrorCode::InvalidState, "lift hardware is not configured"};
    if (!impl_->config.lift_calibration_mode || !impl_->metricLift())
        return {ErrorCode::InvalidState,
                "persistent lift zero is available only in metric calibration mode"};
    if (!impl_->opened || !impl_->lift_attached)
        return {ErrorCode::InvalidState,
                "persistent lift zero requires an open auxiliary CAN connection"};
    if (impl_->lift_worker.joinable())
        return {ErrorCode::Busy, "fixed auxiliary lift motion is active"};

    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    if (impl_->lift_ready)
        return {ErrorCode::InvalidState,
                "disable the lift before setting its persistent zero"};
    if (impl_->chassis_ready)
        return {ErrorCode::InvalidState,
                "disable the chassis before setting the lift persistent zero"};

    const uint8_t node = impl_->config.lift_node_id;
    auto read_object = [&](uint16_t index, uint8_t sub, uint32_t& value,
                           const std::string& label,
                           double timeout_s = 0.8) -> Status {
        const auto result = impl_->lift.sdoRead(node, index, sub, timeout_s);
        if (!result.success) {
            std::ostringstream message;
            message << label << " read failed";
            if (result.aborted)
                message << " (SDO abort 0x" << std::hex << result.abort_code << ')';
            return {ErrorCode::TransportError, message.str()};
        }
        value = result.value;
        return Status::ok();
    };
    auto write_object = [&](uint16_t index, uint8_t sub, uint32_t value,
                            uint8_t size, const std::string& label,
                            double timeout_s = 0.8) -> Status {
        const auto result = impl_->lift.sdoWrite(
            node, index, sub, value, size, timeout_s);
        if (!result.success) {
            std::ostringstream message;
            message << label << " write failed";
            if (result.aborted)
                message << " (SDO abort 0x" << std::hex << result.abort_code << ')';
            return {ErrorCode::TransportError, message.str()};
        }
        return Status::ok();
    };

    uint32_t original_mode = 0;
    bool original_mode_known = false;
    bool mode_changed = false;
    uint16_t verified_statusword = 0;
    const Status operation = [&]() -> Status {
        uint32_t error_code = 0;
        uint32_t statusword = 0;
        uint32_t home_selection = 0;
        uint32_t store_support = 0;
        Status status = read_object(
            elmue_can::canopen::OD_ERROR_CODE, 0, error_code, "0x603F error code");
        if (!status) return status;
        status = read_object(
            elmue_can::canopen::OD_STATUSWORD, 0, statusword, "0x6041 statusword");
        if (!status) return status;
        status = read_object(
            kLiftVendorControlModeObject, kLiftVendorControlModeSub,
            original_mode, "H02-00 control mode");
        if (!status) return status;
        original_mode_known = true;
        status = read_object(
            kLiftVendorHomeObject, kLiftVendorHomeSelectionSub,
            home_selection, "H05-30 home selection");
        if (!status) return status;
        status = read_object(
            elmue_can::canopen::OD_STORE_PARAMS, 1,
            store_support, "0x1010:01 store support");
        if (!status) return status;

        if (error_code != 0 || (statusword & 0x0008u) != 0)
            return {ErrorCode::MotorFault,
                    "lift drive reports a fault; clear it before setting zero"};
        if ((statusword & 0x0004u) != 0)
            return {ErrorCode::InvalidState,
                    "lift drive is still operation-enabled; disable it before setting zero"};
        if (original_mode != kLiftCanopenMode)
            return {ErrorCode::InvalidState,
                    "H02-00 is not the expected CANopen mode 8"};
        if (home_selection != 0)
            return {ErrorCode::Busy, "H05-30 home selection is already busy"};
        if (store_support != 1 && store_support != 3)
            return {ErrorCode::Unsupported,
                    "0x1010:01 does not advertise parameter-save support"};

        int32_t minimum_position = std::numeric_limits<int32_t>::max();
        int32_t maximum_position = std::numeric_limits<int32_t>::min();
        int32_t maximum_abs_velocity = 0;
        for (int sample = 0; sample < kLiftZeroStationarySamples; ++sample) {
            uint32_t raw_position = 0;
            uint32_t raw_velocity = 0;
            status = read_object(
                elmue_can::canopen::OD_POSITION_ACTUAL, 0, raw_position,
                "0x6064 position");
            if (!status) return status;
            status = read_object(
                elmue_can::canopen::OD_VELOCITY_ACTUAL, 0, raw_velocity,
                "0x606C velocity");
            if (!status) return status;
            const int32_t position = static_cast<int32_t>(raw_position);
            const int32_t velocity = static_cast<int32_t>(raw_velocity);
            minimum_position = std::min(minimum_position, position);
            maximum_position = std::max(maximum_position, position);
            maximum_abs_velocity = std::max(
                maximum_abs_velocity,
                static_cast<int32_t>(std::min<int64_t>(
                    std::abs(static_cast<int64_t>(velocity)),
                    std::numeric_limits<int32_t>::max())));
            if (sample + 1 < kLiftZeroStationarySamples)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (static_cast<int64_t>(maximum_position) - minimum_position >
            kLiftZeroStationaryJitterCounts)
            return {ErrorCode::InvalidState,
                    "lift position is not stationary enough to set zero"};
        if (maximum_abs_velocity > kLiftZeroStationaryVelocityCountsS)
            return {ErrorCode::InvalidState,
                    "lift velocity is not stationary enough to set zero"};

        if (!impl_->lift.nmtEnterPreOp(node))
            return {ErrorCode::TransportError,
                    "failed to enter NMT pre-operational before lift zero"};
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!impl_->lift.setControlword(node, 0))
            return {ErrorCode::TransportError,
                    "failed to write disabled controlword before lift zero"};
        bool disabled = false;
        const auto disable_deadline = Clock::now() + std::chrono::seconds(2);
        while (Clock::now() < disable_deadline) {
            status = read_object(
                elmue_can::canopen::OD_STATUSWORD, 0, statusword,
                "0x6041 disabled statusword");
            if (!status) return status;
            if ((statusword & 0x0008u) != 0)
                return {ErrorCode::MotorFault,
                        "lift fault appeared while preparing persistent zero"};
            if ((statusword & 0x0004u) == 0) {
                disabled = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!disabled)
            return {ErrorCode::Timeout,
                    "lift remained operation-enabled while preparing zero"};

        status = write_object(
            kLiftVendorControlModeObject, kLiftVendorControlModeSub,
            kLiftTemporaryPositionMode, 2, "H02-00 temporary position mode");
        if (!status) return status;
        mode_changed = true;
        uint32_t readback = 0;
        status = read_object(
            kLiftVendorControlModeObject, kLiftVendorControlModeSub,
            readback, "H02-00 temporary mode readback");
        if (!status) return status;
        if (readback != kLiftTemporaryPositionMode)
            return {ErrorCode::TransportError,
                    "H02-00 temporary position-mode readback mismatch"};

        status = write_object(
            kLiftVendorPersistObject, kLiftVendorPersistGateSub, 1, 2,
            "0x200C:0E vendor persistence gate");
        if (!status) return status;
        status = write_object(
            kLiftVendorHomeObject, kLiftVendorHomeSelectionSub,
            kLiftSetCurrentAsHome, 2, "H05-30 set-current-as-zero");
        if (!status) return status;

        bool zero_verified = false;
        const auto zero_deadline = Clock::now() + std::chrono::seconds(5);
        while (Clock::now() < zero_deadline) {
            uint32_t raw_position = 0;
            uint32_t raw_multiturn = 0;
            uint32_t raw_turns = 0;
            status = read_object(
                kLiftVendorHomeObject, kLiftVendorHomeSelectionSub,
                home_selection, "H05-30 zero completion");
            if (!status) return status;
            status = read_object(
                elmue_can::canopen::OD_POSITION_ACTUAL, 0,
                raw_position, "0x6064 zero verification");
            if (!status) return status;
            status = read_object(
                kLiftVendorMonitorObject, kLiftVendorMultiturnPositionSub,
                raw_multiturn, "0x200B:08 multi-turn zero verification");
            if (!status) return status;
            status = read_object(
                kLiftVendorMonitorObject, kLiftVendorEncoderTurnsSub,
                raw_turns, "0x200B:47 encoder-turn zero verification");
            if (!status) return status;
            const int32_t position = static_cast<int32_t>(raw_position);
            const int32_t multiturn = static_cast<int32_t>(raw_multiturn);
            const int32_t turns = static_cast<int32_t>(raw_turns);
            if (home_selection == 0 &&
                std::abs(static_cast<int64_t>(position)) <=
                    kLiftZeroPositionToleranceCounts &&
                std::abs(static_cast<int64_t>(multiturn)) <=
                    kLiftZeroPositionToleranceCounts &&
                turns == 0) {
                zero_verified = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!zero_verified)
            return {ErrorCode::Timeout,
                    "lift multi-turn zero did not verify before timeout"};

        status = write_object(
            kLiftVendorControlModeObject, kLiftVendorControlModeSub,
            original_mode, 2, "H02-00 restore CANopen mode");
        if (!status) return status;
        status = read_object(
            kLiftVendorControlModeObject, kLiftVendorControlModeSub,
            readback, "H02-00 restored-mode readback");
        if (!status) return status;
        if (readback != original_mode)
            return {ErrorCode::TransportError,
                    "H02-00 CANopen-mode restore readback mismatch"};
        mode_changed = false;

        status = write_object(
            elmue_can::canopen::OD_STORE_PARAMS, 1, kCanopenStoreSave, 4,
            "0x1010:01 parameter save", 2.0);
        if (!status) return status;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        if (!impl_->lift.nmtResetNode(node))
            return {ErrorCode::TransportError,
                    "failed to reset lift node after saving zero"};
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        if (!impl_->lift.nmtEnterPreOp(node))
            return {ErrorCode::TransportError,
                    "failed to return reset lift node to pre-operational"};
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        uint32_t reset_mode = 0;
        uint32_t reset_position = 0;
        uint32_t reset_multiturn = 0;
        uint32_t reset_turns = 0;
        status = read_object(
            kLiftVendorControlModeObject, kLiftVendorControlModeSub,
            reset_mode, "post-reset H02-00 mode");
        if (!status) return status;
        status = read_object(
            elmue_can::canopen::OD_POSITION_ACTUAL, 0,
            reset_position, "post-reset 0x6064 position");
        if (!status) return status;
        status = read_object(
            kLiftVendorMonitorObject, kLiftVendorMultiturnPositionSub,
            reset_multiturn, "post-reset 0x200B:08 multi-turn position");
        if (!status) return status;
        status = read_object(
            kLiftVendorMonitorObject, kLiftVendorEncoderTurnsSub,
            reset_turns, "post-reset 0x200B:47 encoder turns");
        if (!status) return status;
        status = read_object(
            elmue_can::canopen::OD_STATUSWORD, 0, statusword,
            "post-reset 0x6041 statusword");
        if (!status) return status;
        if (reset_mode != original_mode ||
            std::abs(static_cast<int64_t>(
                static_cast<int32_t>(reset_position))) >
                kLiftZeroPositionToleranceCounts ||
            std::abs(static_cast<int64_t>(
                static_cast<int32_t>(reset_multiturn))) >
                kLiftZeroPositionToleranceCounts ||
            static_cast<int32_t>(reset_turns) != 0)
            return {ErrorCode::TransportError,
                    "saved lift zero failed post-reset verification"};
        verified_statusword = static_cast<uint16_t>(statusword);
        return Status::ok();
    }();

    Status restore_status = Status::ok();
    if (mode_changed && original_mode_known) {
        restore_status = write_object(
            kLiftVendorControlModeObject, kLiftVendorControlModeSub,
            original_mode, 2, "emergency H02-00 CANopen-mode restore");
        if (restore_status) {
            uint32_t restored_mode = 0;
            restore_status = read_object(
                kLiftVendorControlModeObject, kLiftVendorControlModeSub,
                restored_mode, "emergency H02-00 restored-mode readback");
            if (restore_status && restored_mode != original_mode)
                restore_status = {
                    ErrorCode::TransportError,
                    "emergency H02-00 CANopen-mode restore readback mismatch"};
            if (restore_status) mode_changed = false;
        }
    }
    (void)impl_->lift.setControlword(node, 0);
    (void)impl_->lift.nmtEnterPreOp(node);
    if (!restore_status) {
        return {ErrorCode::TransportError,
                operation.message() +
                    "; FATAL: H02-00 restore also failed; keep the lift stopped"};
    }
    if (!operation) return operation;

    impl_->programmed_lift_profile_velocity_mm_s.reset();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_enabled = false;
        impl_->snapshot.lift_position_counts = 0;
        impl_->snapshot.lift_position_mm = 0.0;
        impl_->snapshot.lift_position_valid = true;
        impl_->snapshot.lift_velocity_counts_s = 0.0;
        impl_->snapshot.lift_velocity_mm_s = 0.0;
        impl_->snapshot.lift_velocity_valid = true;
        impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        impl_->snapshot.lift_target_valid = false;
        impl_->snapshot.lift_target_reached = false;
        impl_->snapshot.lift_halted = false;
        impl_->snapshot.lift_statusword = verified_statusword;
        impl_->snapshot.lift_error_code = 0;
        impl_->snapshot.motion_running = false;
    }
    return Status::ok();
}

Status AuxiliaryController::refreshLiftFeedback() {
    if (!impl_->config.enable_lift || !impl_->opened || !impl_->lift_ready)
        return {ErrorCode::InvalidState,
                "lift feedback refresh requires an enabled drive"};
    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    if (!impl_->refreshLiftMotionFeedback())
        return {ErrorCode::FeedbackStale,
                "lift position/velocity feedback refresh failed"};

    double commanded = 0.0;
    bool outward_limit = false;
    uint16_t error_code = 0;
    uint16_t statusword = 0;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        commanded = impl_->snapshot.lift_commanded_velocity_counts_s;
        outward_limit = !impl_->metricLift() &&
            ((commanded > 0.0 && impl_->snapshot.lift_upper_limit_active) ||
             (commanded < 0.0 && impl_->snapshot.lift_lower_limit_active));
        error_code = impl_->snapshot.lift_error_code;
        statusword = impl_->snapshot.lift_statusword;
    }
    const auto drive_state = elmue_can::canopen::parseState(statusword);
    const bool drive_faulted =
        drive_state == elmue_can::canopen::Cia402State::Fault ||
        drive_state == elmue_can::canopen::Cia402State::FaultReactionActive ||
        drive_state == elmue_can::canopen::Cia402State::QuickStopActive;
    if (error_code != 0 || drive_faulted) {
        (void)impl_->stopLiftDevice();
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        impl_->snapshot.lift_halted = impl_->metricLift();
        impl_->snapshot.lift_target_reached = false;
        return {ErrorCode::MotorFault,
                error_code != 0
                    ? "lift CANopen error code is nonzero"
                    : "lift CANopen statusword reports fault or quick-stop state"};
    }
    if (impl_->lift_ready &&
        drive_state != elmue_can::canopen::Cia402State::OperationEnabled) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        impl_->snapshot.lift_target_reached = false;
        return {ErrorCode::InvalidState,
                "lift lost OperationEnabled: " +
                std::string(elmue_can::canopen::stateName(drive_state)) +
                " statusword=" + std::to_string(statusword) +
                "; no further position targets; inspect drive enable/STO/power"};
    }
    if (outward_limit) {
        const bool stopped = impl_->stopLiftDevice();
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        }
        return {stopped ? ErrorCode::LimitViolation : ErrorCode::TransportError,
                stopped ? "lift reached the relative software limit"
                        : "lift reached the software limit and zero write failed"};
    }
    return Status::ok();
}

Status AuxiliaryController::disable() {
    requestStop();
    // Interrupted motion is expected during lifecycle shutdown; always proceed
    // with zero velocity and drive disable, while retaining the state reason.
    (void)waitMotion();
    std::lock_guard<std::mutex> io_lock(impl_->io_mutex);
    bool ok = true;
    if (impl_->chassis_ready) ok = impl_->sendAllWheelVelocities(0.0) && ok;
    if (impl_->lift_attached)
        if (impl_->lift_ready) ok = impl_->stopLiftDevice() && ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (impl_->lift_attached) {
        if (impl_->lift_ready) ok = impl_->lift.disable(impl_->config.lift_node_id) && ok;
        ok = impl_->lift.nmtEnterPreOp(impl_->config.lift_node_id) && ok;
    }
    if (impl_->config.enable_chassis) {
        for (uint8_t id : kChassisMotorIds) {
            ok = impl_->chassis.setActiveReport(id, false) && ok;
            ok = impl_->chassis.disableMotor(id, false) && ok;
        }
    }
    impl_->lift_ready = false;
    impl_->programmed_lift_profile_velocity_mm_s.reset();
    impl_->chassis_ready = false;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->snapshot.enabled = false;
        impl_->snapshot.chassis_enabled = false;
        impl_->snapshot.chassis_velocity_feedback_valid = false;
        impl_->snapshot.lift_enabled = false;
        impl_->snapshot.motion_running = false;
        impl_->snapshot.chassis_commanded_velocities.fill(0.0);
        impl_->snapshot.lift_commanded_velocity_counts_s = 0.0;
        impl_->snapshot.lift_limits_valid = false;
        impl_->snapshot.lift_lower_limit_active = false;
        impl_->snapshot.lift_upper_limit_active = false;
    }
    return ok ? Status::ok()
              : Status{ErrorCode::TransportError, "one or more auxiliary devices failed to disable"};
}

void AuxiliaryController::close() {
    if (!impl_ || !impl_->opened) return;
    (void)disable();
    if (impl_->lift_attached) {
        impl_->chassis.setForeignStdFrameCallback({});
        impl_->lift.close();
    }
    impl_->chassis.close();
    impl_->opened = false;
    impl_->lift_attached = false;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->snapshot.connected = false;
}

AuxiliaryState AuxiliaryController::state() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->snapshot;
}

}  // namespace poke::detail
