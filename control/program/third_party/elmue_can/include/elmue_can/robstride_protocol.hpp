// robstride_protocol.hpp —— Robstride 私有协议：传输无关的类型、量程与帧 payload 编解码
//
// C++ 侧单一事实来源 (SSOT)。ElmueCan / SocketCAN 驱动仅负责收发 CAN 帧，
// 本头文件提供 MotorType、MotorParams、float↔uint16 映射及运控/反馈 payload 打包解析。

#ifndef ELMUE_CAN__ROBSTRIDE_PROTOCOL_HPP_
#define ELMUE_CAN__ROBSTRIDE_PROTOCOL_HPP_

#include <chrono>
#include <cstdint>

namespace elmue_can {

enum class MotorType {
    RS00 = 0,
    EL05 = 1,
    RS05 = 2,
    RS03 = 3,
    RS04 = 4,
    // 达妙 DM4340（MIT 标准帧协议，与 Robstride 私有协议不同，见 ProtocolFamily）
    DM4340 = 5,      // 24V: P=±12.5, V=±8,  T=±28
    DM4340_48V = 6,  // 48V: P=±12.5, V=±10, T=±28
    RS10 = 7,
    // RobStride RS02 chassis motor. The protocol manual specifies the same
    // position/Kp/Kd fields as RS00, with V=±44 rad/s and T=±17 N*m.
    RS02 = 8,
};

/**
 * @brief 协议族：决定驱动层用哪条编解码/收发路径
 *   ROBSTRIDE  —— 扩展帧(29位) + 4×16 打包 + comm_type 指令
 *   DAMIAO_MIT —— 标准帧(11位) + MIT 16/12/12/12/12 打包 + FF..FC/FD/FE 指令
 */
enum class ProtocolFamily {
    ROBSTRIDE = 0,
    DAMIAO_MIT = 1,
};

enum class RunMode : uint8_t {
    MOTION_CONTROL = 0,
    POSITION_PP = 1,
    VELOCITY = 2,
    CURRENT = 3,
    POSITION_CSP = 5,
};

struct MotorParams {
    double p_min;
    double p_max;
    double v_min;
    double v_max;
    double t_min;
    double t_max;
    double kp_min;
    double kp_max;
    double kd_min;
    double kd_max;
};

struct MotorFeedback {
    uint8_t motor_id = 0;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double temperature = 0.0;
    uint8_t mode_state = 0;
    uint8_t fault_code = 0;
    bool is_valid = false;
    std::chrono::steady_clock::time_point last_update{};
};

// Robstride 参数索引
namespace robstride {
constexpr uint16_t PARAM_RUN_MODE = 0x7005;
constexpr uint16_t PARAM_LOC_REF = 0x7016;
constexpr uint16_t PARAM_LIMIT_SPD = 0x7017;
constexpr uint16_t PARAM_LIMIT_CUR = 0x7018;
constexpr uint16_t PARAM_VEL_MAX = 0x7024;
constexpr uint16_t PARAM_ACC_SET = 0x7025;
}  // namespace robstride

// 达妙 MIT 协议常量
namespace damiao {
// MIT 控制帧第 8 字节（命令字节）/ 特殊指令尾字节
constexpr uint8_t CMD_ENABLE = 0xFC;    // 使能：data = FF FF FF FF FF FF FF FC
constexpr uint8_t CMD_DISABLE = 0xFD;   // 失能：data = FF FF FF FF FF FF FF FD
constexpr uint8_t CMD_SET_ZERO = 0xFE;  // 置零：data = FF FF FF FF FF FF FF FE
// MIT 运控 Kp/Kd 映射范围（各型号一致）
constexpr double KP_MIN = 0.0;
constexpr double KP_MAX = 500.0;
constexpr double KD_MIN = 0.0;
constexpr double KD_MAX = 5.0;
}  // namespace damiao

MotorParams getMotorParams(MotorType type);
const char *motorTypeName(MotorType type);
ProtocolFamily protocolFamily(MotorType type);

uint16_t floatToUint16(double x, double x_min, double x_max);
double uint16ToFloat(uint16_t x_int, double x_min, double x_max);

/** 通用 N 位（1~16）定点映射，用于达妙 MIT 12/16 位字段 */
uint16_t floatToUintN(double x, double x_min, double x_max, int bits);
double uintNToFloat(uint16_t x_int, double x_min, double x_max, int bits);

/** 构造 29 位扩展 CAN ID（不含 SocketCAN EFF 标志） */
uint32_t buildExtendedCanId(uint8_t comm_type, uint16_t data_area2, uint8_t target_id);

/**
 * 打包运控 Type1 payload（8 字节）及 torque_raw（用于 CAN ID data_area2）
 * @return torque_raw 写入 CAN ID bit23~8
 */
uint16_t packMotionControlPayload(MotorType motor_type,
                                  double position,
                                  double velocity,
                                  double kp,
                                  double kd,
                                  double torque,
                                  uint8_t data[8]);

/** 解析 Type2 反馈 payload；mode_state/fault_code 从 can_id 高位提取 */
MotorFeedback parseFeedbackPayload(uint32_t can_id,
                                   const uint8_t data[8],
                                   MotorType motor_type);

/**
 * 打包达妙 MIT 运控 payload（8 字节）：
 *   pos 16 位(±PMAX) | vel 12 位(±VMAX) | kp 12 位(0~500) | kd 12 位(0~5) | tau 12 位(±TMAX)
 * 与 Robstride 不同：力矩在 payload 内（非 CAN ID），速度按电机 VMAX 而非固定 ±50。
 */
void packMitControlPayload(MotorType motor_type,
                           double position,
                           double velocity,
                           double kp,
                           double kd,
                           double torque,
                           uint8_t data[8]);

/**
 * 解析达妙 MIT 反馈 payload（标准帧 8 字节）：
 *   D0=ID|ERR<<4, D1~2=POS(16), D3~D4hi=VEL(12), D4lo~D5=T(12), D6=T_MOS, D7=T_Rotor
 * motor_id 由调用方按 recv_id(MST_ID) 映射传入；fault_code 存 ERR、temperature 取转子温度。
 */
MotorFeedback parseMitFeedbackPayload(const uint8_t data[8],
                                      MotorType motor_type,
                                      uint8_t motor_id);

}  // namespace elmue_can

#endif  // ELMUE_CAN__ROBSTRIDE_PROTOCOL_HPP_
