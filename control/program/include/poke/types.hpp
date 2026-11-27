#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "poke/status.hpp"
namespace poke {
enum class LiftControlMode {
    RawProfileVelocity,
    MetricProfilePosition,
};

// Software configuration ceiling for each chassis wheel. The deployed chassis
// uses RS02 protocol speed range +/-44 rad/s. This 25 rad/s software
// ceiling supports the 1.5 m/s policy; it is not hardware qualification.
inline constexpr double kMaximumChassisWheelSpeedRadS = 25.0;

// Reviewed absolute lift travel. Applications may configure a narrower
// interval, but metric Profile Position must never extend above this ceiling.
inline constexpr double kMaximumLiftPositionMm = 250.0;

struct AuxiliaryConfig {
    std::string serial;
    bool enable_lift{false};
    bool enable_chassis{false};
    // Backward-compatible default for existing standalone examples. Services
    // that expose independent auxiliary lifecycle set this false and call the
    // device-specific enable API explicitly after startControl().
    bool enable_on_robot_enable{true};
    uint8_t lift_node_id{1};
    LiftControlMode lift_control_mode{LiftControlMode::RawProfileVelocity};
    double lift_speed_counts_s{1000.0};
    // Symmetric session-only range around the enable position. Zero disables
    // this software displacement guard; it does not imply a hardware endstop.
    int32_t lift_travel_counts{2000};
    // Metric Profile Position parameters. For a reduction ratio of 7, the
    // motor turns seven revolutions for one screw revolution. These fields are
    // required only when lift_control_mode is MetricProfilePosition.
    double lift_motor_counts_per_revolution{0.0};
    double lift_reduction_ratio{0.0};
    double lift_screw_lead_mm_per_revolution{0.0};
    double lift_min_position_mm{0.0};
    double lift_max_position_mm{0.0};
    double lift_max_profile_velocity_mm_s{0.0};
    double lift_position_tolerance_mm{0.5};
    // Explicit commissioning exception for recovering an invalid persistent
    // lift coordinate. It may hold an out-of-range measured position and
    // accepts only bounded relative PP jog segments at the reduced speed cap.
    bool lift_calibration_mode{false};
    double lift_calibration_max_profile_velocity_mm_s{3.0};
    double lift_calibration_jog_span_mm{100.0};
    double liftCountsPerMm() const {
        if (lift_screw_lead_mm_per_revolution == 0.0) return 0.0;
        return lift_motor_counts_per_revolution * lift_reduction_ratio /
               lift_screw_lead_mm_per_revolution;
    }
    double chassis_speed_rad_s{0.8};
    // RS02 type-24 reports every 10 ms by default. Cached chassis velocity
    // feedback older than this bound is rejected by the public refresh API.
    uint32_t chassis_feedback_timeout_ms{100};
};

struct AuxiliaryState {
    bool configured{false};
    bool connected{false};
    bool enabled{false};
    bool chassis_configured{false};
    bool lift_configured{false};
    bool chassis_enabled{false};
    bool lift_enabled{false};
    bool motion_running{false};
    bool stop_requested{false};
    int32_t lift_position_counts{0};
    bool lift_position_valid{false};
    double lift_velocity_counts_s{0.0};
    bool lift_velocity_valid{false};
    double lift_commanded_velocity_counts_s{0.0};
    bool lift_metric_position_configured{false};
    double lift_counts_per_mm{0.0};
    double lift_position_mm{0.0};
    double lift_velocity_mm_s{0.0};
    double lift_target_position_mm{0.0};
    double lift_profile_velocity_mm_s{0.0};
    double lift_min_position_mm{0.0};
    double lift_max_position_mm{0.0};
    bool lift_target_valid{false};
    bool lift_target_reached{false};
    bool lift_halted{false};
    int32_t lift_origin_counts{0};
    int32_t lift_minimum_counts{0};
    int32_t lift_maximum_counts{0};
    bool lift_limits_valid{false};
    bool lift_lower_limit_active{false};
    bool lift_upper_limit_active{false};
    uint16_t lift_statusword{0};
    uint16_t lift_error_code{0};
    std::array<double, 4> chassis_positions{};
    bool chassis_positions_valid{false};
    std::array<double, 4> chassis_velocities{};
    std::array<double, 4> chassis_temperatures{};
    std::array<uint8_t, 4> chassis_fault_codes{};
    bool chassis_velocity_feedback_valid{false};
    double chassis_velocity_feedback_max_age_ms{0.0};
    std::array<double, 4> chassis_commanded_velocities{};
    std::string fault_reason;
};


} // namespace poke
