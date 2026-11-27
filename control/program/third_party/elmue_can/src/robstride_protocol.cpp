// robstride_protocol.cpp —— Robstride 协议 SSOT 实现

#include "elmue_can/robstride_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace elmue_can {

MotorParams getMotorParams(MotorType type) {
    MotorParams params{};
    params.p_min = -12.57;
    params.p_max = 12.57;
    params.kp_min = 0.0;
    params.kp_max = 500.0;
    params.kd_min = 0.0;
    params.kd_max = 5.0;

    switch (type) {
        case MotorType::RS00:
            params.v_min = -33.0;
            params.v_max = 33.0;
            params.t_min = -14.0;
            params.t_max = 14.0;
            break;
        case MotorType::RS02:
            params.v_min = -44.0;
            params.v_max = 44.0;
            params.t_min = -17.0;
            params.t_max = 17.0;
            break;
        case MotorType::EL05:
            params.v_min = -50.0;
            params.v_max = 50.0;
            params.t_min = -6.0;
            params.t_max = 6.0;
            break;
        case MotorType::RS05:
            params.v_min = -50.0;
            params.v_max = 50.0;
            params.t_min = -5.5;
            params.t_max = 5.5;
            break;
        case MotorType::RS03:
            // RS03 Type 1 command and Type 2 feedback fields use the same
            // ranges documented by RobStride: V=+/-20 rad/s, T=+/-60 N*m,
            // Kp=0..5000 and Kd=0..100.
            params.v_min = -20.0;
            params.v_max = 20.0;
            // RS03 communication types 1/2 encode command and feedback
            // torque over the actuator's +/-60 N*m peak range. 20 N*m is
            // the rated torque, not the protocol field range.
            params.t_min = -60.0;
            params.t_max = 60.0;
            params.kp_max = 5000.0;
            params.kd_max = 100.0;  // RS03 固件 Kd 量程 0~100
            break;
        case MotorType::RS04:
            // RS04 Type 1 command and Type 2 feedback fields: V=+/-15
            // rad/s, T=+/-120 N*m, Kp=0..5000 and Kd=0..100.
            params.v_min = -15.0;
            params.v_max = 15.0;
            params.t_min = -120.0;
            params.t_max = 120.0;
            params.kp_max = 5000.0;
            params.kd_max = 100.0;  // RS04 固件 Kd 量程 0~100
            break;
        case MotorType::RS10:
            params.v_min = -13.0;
            params.v_max = 13.0;
            params.t_min = -37.0;
            params.t_max = 37.0;
            break;
        case MotorType::DM4340:
            // 达妙 DM4340 24V：P=±12.5, V=±8, T=±28；MIT Kp[0,500] Kd[0,5]
            params.p_min = -12.5;
            params.p_max = 12.5;
            params.v_min = -8.0;
            params.v_max = 8.0;
            params.t_min = -28.0;
            params.t_max = 28.0;
            break;
        case MotorType::DM4340_48V:
            // 达妙 DM4340 48V：P=±12.5, V=±10, T=±28
            params.p_min = -12.5;
            params.p_max = 12.5;
            params.v_min = -10.0;
            params.v_max = 10.0;
            params.t_min = -28.0;
            params.t_max = 28.0;
            break;
    }
    return params;
}

const char *motorTypeName(MotorType type) {
    switch (type) {
        case MotorType::RS00: return "RS00";
        case MotorType::RS02: return "RS02";
        case MotorType::EL05: return "EL05";
        case MotorType::RS05: return "RS05";
        case MotorType::RS03: return "RS03";
        case MotorType::RS04: return "RS04";
        case MotorType::RS10: return "RS10";
        case MotorType::DM4340: return "DM4340";
        case MotorType::DM4340_48V: return "DM4340_48V";
        default: return "UNKNOWN";
    }
}

ProtocolFamily protocolFamily(MotorType type) {
    switch (type) {
        case MotorType::DM4340:
        case MotorType::DM4340_48V:
            return ProtocolFamily::DAMIAO_MIT;
        default:
            return ProtocolFamily::ROBSTRIDE;
    }
}

uint16_t floatToUint16(double x, double x_min, double x_max) {
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    const double span = x_max - x_min;
    const double raw = (x - x_min) * 65535.0 / span + 0.5;
    if (raw < 0.0) return 0;
    if (raw > 65535.0) return 65535;
    return static_cast<uint16_t>(raw);
}

double uint16ToFloat(uint16_t x_int, double x_min, double x_max) {
    const double span = x_max - x_min;
    return static_cast<double>(x_int) * span / 65535.0 + x_min;
}

uint16_t floatToUintN(double x, double x_min, double x_max, int bits) {
    if (bits <= 0) return 0;
    if (bits > 16) bits = 16;
    const uint32_t max_int = (1u << bits) - 1u;
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    const double span = x_max - x_min;
    if (span <= 0.0) return 0;
    double raw = (x - x_min) * static_cast<double>(max_int) / span + 0.5;
    if (raw < 0.0) raw = 0.0;
    if (raw > static_cast<double>(max_int)) raw = static_cast<double>(max_int);
    return static_cast<uint16_t>(raw);
}

double uintNToFloat(uint16_t x_int, double x_min, double x_max, int bits) {
    if (bits <= 0) return x_min;
    if (bits > 16) bits = 16;
    const uint32_t max_int = (1u << bits) - 1u;
    const double span = x_max - x_min;
    return static_cast<double>(x_int) * span / static_cast<double>(max_int) + x_min;
}

uint32_t buildExtendedCanId(uint8_t comm_type, uint16_t data_area2, uint8_t target_id) {
    return ((static_cast<uint32_t>(comm_type) & 0x1F) << 24) |
           ((static_cast<uint32_t>(data_area2) & 0xFFFF) << 8) |
           (static_cast<uint32_t>(target_id) & 0xFF);
}

uint16_t packMotionControlPayload(MotorType motor_type,
                                  double position,
                                  double velocity,
                                  double kp,
                                  double kd,
                                  double torque,
                                  uint8_t data[8]) {
    const MotorParams params = getMotorParams(motor_type);
    const uint16_t torque_raw = floatToUint16(torque, params.t_min, params.t_max);
    const uint16_t pos_raw = floatToUint16(position, params.p_min, params.p_max);
    // RobStride Type 1 velocity is model-specific. Reusing a global +/-50
    // range silently changes the physical meaning of every non-zero command.
    const uint16_t vel_raw = floatToUint16(velocity, params.v_min, params.v_max);
    const uint16_t kp_raw = floatToUint16(kp, params.kp_min, params.kp_max);
    const uint16_t kd_raw = floatToUint16(kd, params.kd_min, params.kd_max);

    data[0] = (pos_raw >> 8) & 0xFF;
    data[1] = pos_raw & 0xFF;
    data[2] = (vel_raw >> 8) & 0xFF;
    data[3] = vel_raw & 0xFF;
    data[4] = (kp_raw >> 8) & 0xFF;
    data[5] = kp_raw & 0xFF;
    data[6] = (kd_raw >> 8) & 0xFF;
    data[7] = kd_raw & 0xFF;
    return torque_raw;
}

MotorFeedback parseFeedbackPayload(uint32_t can_id,
                                   const uint8_t data[8],
                                   MotorType motor_type) {
    const MotorParams params = getMotorParams(motor_type);
    const uint8_t motor_id = (can_id >> 8) & 0xFF;

    const uint16_t pos_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const uint16_t vel_raw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    const uint16_t torque_raw = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    const uint16_t temp_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];

    MotorFeedback fb{};
    fb.motor_id = motor_id;
    fb.position = uint16ToFloat(pos_raw, params.p_min, params.p_max);
    fb.velocity = uint16ToFloat(vel_raw, params.v_min, params.v_max);
    fb.torque = uint16ToFloat(torque_raw, params.t_min, params.t_max);
    fb.temperature = static_cast<double>(temp_raw) / 10.0;
    fb.mode_state = (can_id >> 22) & 0x03;
    fb.fault_code = (can_id >> 16) & 0x3F;
    fb.is_valid = true;
    fb.last_update = std::chrono::steady_clock::now();
    return fb;
}

void packMitControlPayload(MotorType motor_type,
                           double position,
                           double velocity,
                           double kp,
                           double kd,
                           double torque,
                           uint8_t data[8]) {
    const MotorParams params = getMotorParams(motor_type);
    const uint16_t pos_raw = floatToUintN(position, params.p_min, params.p_max, 16);
    const uint16_t vel_raw = floatToUintN(velocity, params.v_min, params.v_max, 12);
    const uint16_t kp_raw = floatToUintN(kp, damiao::KP_MIN, damiao::KP_MAX, 12);
    const uint16_t kd_raw = floatToUintN(kd, damiao::KD_MIN, damiao::KD_MAX, 12);
    const uint16_t tau_raw = floatToUintN(torque, params.t_min, params.t_max, 12);

    data[0] = (pos_raw >> 8) & 0xFF;
    data[1] = pos_raw & 0xFF;
    data[2] = (vel_raw >> 4) & 0xFF;
    data[3] = ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F);
    data[4] = kp_raw & 0xFF;
    data[5] = (kd_raw >> 4) & 0xFF;
    data[6] = ((kd_raw & 0x0F) << 4) | ((tau_raw >> 8) & 0x0F);
    data[7] = tau_raw & 0xFF;
}

MotorFeedback parseMitFeedbackPayload(const uint8_t data[8],
                                      MotorType motor_type,
                                      uint8_t motor_id) {
    const MotorParams params = getMotorParams(motor_type);
    const uint16_t pos_raw = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    const uint16_t vel_raw = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    const uint16_t tau_raw = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];
    const uint8_t err = (data[0] >> 4) & 0x0F;

    MotorFeedback fb{};
    fb.motor_id = motor_id;
    fb.position = uintNToFloat(pos_raw, params.p_min, params.p_max, 16);
    fb.velocity = uintNToFloat(vel_raw, params.v_min, params.v_max, 12);
    fb.torque = uintNToFloat(tau_raw, params.t_min, params.t_max, 12);
    fb.temperature = static_cast<double>(data[7]);  // 转子绕组温度（℃，直读）
    fb.mode_state = 2;  // 收到 MIT 反馈即视为 MOTOR 运行态
    fb.fault_code = err;  // 达妙 ERR 半字节（8:过压 9:欠压 A:过流 B:MOS过温 C:转子过温…）
    fb.is_valid = true;
    fb.last_update = std::chrono::steady_clock::now();
    return fb;
}

}  // namespace elmue_can
