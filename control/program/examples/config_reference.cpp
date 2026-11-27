// Offline configuration reference: never connects to CAN or enables motors.
#include "poke/base.hpp"
#include <iostream>
int main() {
    poke::AuxiliaryConfig aux;
    aux.serial = "REPLACE_WITH_VERIFIED_CAN_SERIAL";
    aux.enable_chassis = true;
    aux.enable_lift = true;
    aux.enable_on_robot_enable = false;
    aux.chassis_speed_rad_s = 1.0;
    aux.chassis_feedback_timeout_ms = 100;
    aux.lift_node_id = 1;
    aux.lift_control_mode = poke::LiftControlMode::MetricProfilePosition;
    aux.lift_travel_counts = 0; // Absolute millimetre limits supersede raw-count guard.
    // Values from the archive documentation, not newly measured on this NUC.
    aux.lift_motor_counts_per_revolution = 800.0;
    aux.lift_reduction_ratio = 7.0;
    aux.lift_screw_lead_mm_per_revolution = 48.0;
    aux.lift_min_position_mm = 0.0;
    aux.lift_max_position_mm = 250.0;
    aux.lift_max_profile_velocity_mm_s = 3.0;
    poke::BaseConfig base;
    base.control_rate_hz = 50.0;
    base.lift_command_rate_hz = 10.0;
    base.chassis.max_wheel_speed_rad_s = aux.chassis_speed_rad_s;
    base.lift_max_speed_m_s = 0.003;
    std::cout << "Configuration only; no hardware connection.\n"
              << "counts/mm: " << aux.liftCountsPerMm() << '\n'
              << "base Hz: " << base.control_rate_hz << '\n'
              << "lift Hz: " << base.lift_command_rate_hz << '\n';
}
