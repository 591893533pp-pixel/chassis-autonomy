// canopen_protocol.hpp —— CANopen (CiA-301) + CiA-402 运动控制子协议：
//   传输无关的常量、SDO/PDO/NMT 编解码、CiA-402 状态机与量纲换算。
//
// C++ 侧单一事实来源 (SSOT)。ElmueCan 驱动仅负责收发经典 CAN 标准帧（11 位 COB-ID，
// 8 字节以内），本头文件提供：
//   - COB-ID 功能码基址（NMT/SYNC/EMCY/PDO/SDO/Heartbeat）
//   - NMT 主站命令、SDO 客户端命令字与 expedited 上传/下载编解码
//   - CiA-402 对象字典索引、Modes of operation、Controlword/Statusword 状态机
//   - device counts <-> SI(rad, rad/s) 量纲换算
//
// 适用从站：ZeroErr eDriver / eRob 系列（EDS: "ZeroErr Driver_V1.5.eds"），
//   以及任何遵循 CiA-402 的 CANopen 伺服（对象 0x6040/0x6041/0x6060/0x6064/... 通用）。

#ifndef ELMUE_CAN__CANOPEN_PROTOCOL_HPP_
#define ELMUE_CAN__CANOPEN_PROTOCOL_HPP_

#include <cmath>
#include <cstdint>

namespace elmue_can {
namespace canopen {

// ============================ COB-ID 功能码基址 ============================
// 预定义连接集 (CiA-301)：COB-ID = 功能码基址 + node_id (1~127)。
// NMT / SYNC 为广播，不加 node_id。
constexpr uint32_t COBID_NMT = 0x000;        // NMT 网络管理（主站广播，dlc=2）
constexpr uint32_t COBID_SYNC = 0x080;       // 同步对象（广播，dlc=0）
constexpr uint32_t COBID_EMCY_BASE = 0x080;  // 紧急报文 (0x80 + id)
constexpr uint32_t COBID_TPDO1_BASE = 0x180; // 从站发（我们收）
constexpr uint32_t COBID_RPDO1_BASE = 0x200; // 主站发（我们发）
constexpr uint32_t COBID_TPDO2_BASE = 0x280;
constexpr uint32_t COBID_RPDO2_BASE = 0x300;
constexpr uint32_t COBID_TPDO3_BASE = 0x380;
constexpr uint32_t COBID_RPDO3_BASE = 0x400;
constexpr uint32_t COBID_TPDO4_BASE = 0x480;
constexpr uint32_t COBID_RPDO4_BASE = 0x500;
constexpr uint32_t COBID_SDO_TX_BASE = 0x580;  // 服务器->客户端（SDO 响应，我们收）
constexpr uint32_t COBID_SDO_RX_BASE = 0x600;  // 客户端->服务器（SDO 请求，我们发）
constexpr uint32_t COBID_HEARTBEAT_BASE = 0x700;  // 心跳 / BootUp (0x700 + id)

// 功能码识别掩码：COB-ID 高 4 位为功能码，低 7 位为 node_id。
constexpr uint32_t FUNCTION_MASK = 0x780;
constexpr uint32_t NODE_ID_MASK = 0x07F;

inline uint32_t cobFunction(uint32_t cob_id) { return cob_id & FUNCTION_MASK; }
inline uint8_t cobNodeId(uint32_t cob_id) { return static_cast<uint8_t>(cob_id & NODE_ID_MASK); }

// ============================ NMT 网络管理 ============================
// NMT 帧：COB-ID=0x000, data=[command, target_node]（target_node=0 表示所有从站）。
enum NmtCommand : uint8_t {
    NMT_START_NODE = 0x01,      // 进入 Operational（PDO 生效）
    NMT_STOP_NODE = 0x02,       // 进入 Stopped
    NMT_ENTER_PREOP = 0x80,     // 进入 Pre-operational（仅 SDO/心跳）
    NMT_RESET_NODE = 0x81,      // 复位应用（重启节点）
    NMT_RESET_COMM = 0x82,      // 复位通信
};

// NMT 从站状态（心跳报文 data[0] 低 7 位）
enum NmtState : uint8_t {
    NMT_STATE_BOOTUP = 0x00,     // 仅 BootUp 报文瞬时值
    NMT_STATE_STOPPED = 0x04,
    NMT_STATE_OPERATIONAL = 0x05,
    NMT_STATE_PREOP = 0x7F,
};

// ============================ SDO 服务数据对象 ============================
// SDO 请求 (客户端->服务器, COB=0x600+id, dlc=8)：
//   byte0=命令字, byte1..2=index(LE), byte3=subindex, byte4..7=data(LE)
// SDO 响应 (服务器->客户端, COB=0x580+id, dlc=8)：同布局，byte0 为响应命令字。
//
// expedited（≤4 字节，一次往返完成）命令字：
enum SdoCommand : uint8_t {
    // 下载（写）请求：0x2n，n 编码字节数（size-specified, expedited）
    SDO_DL_1BYTE = 0x2F,
    SDO_DL_2BYTE = 0x2B,
    SDO_DL_3BYTE = 0x27,
    SDO_DL_4BYTE = 0x23,
    SDO_DL_RESP = 0x60,   // 下载成功响应
    // 上传（读）请求：
    SDO_UL_REQ = 0x40,
    // 上传响应：0x4n（expedited），byte0 低位含 size/expedited 标志
    SDO_UL_1BYTE = 0x4F,
    SDO_UL_2BYTE = 0x4B,
    SDO_UL_3BYTE = 0x47,
    SDO_UL_4BYTE = 0x43,
    SDO_ABORT = 0x80,     // 中止传输，byte4..7=abort code
};

// 按字节数取下载命令字（1~4 字节 expedited）
inline uint8_t sdoDownloadCmd(uint8_t nbytes) {
    switch (nbytes) {
        case 1: return SDO_DL_1BYTE;
        case 2: return SDO_DL_2BYTE;
        case 3: return SDO_DL_3BYTE;
        default: return SDO_DL_4BYTE;
    }
}

// 从上传响应命令字解析出有效数据字节数（expedited）。返回 0 表示非 expedited/异常。
inline uint8_t sdoUploadRespBytes(uint8_t cmd) {
    // 命令字：0100 nnss，e=expedited(bit1), s=size-indicated(bit0)，nn=4-字节数
    if ((cmd & 0xE0) != 0x40) return 0;    // 非上传响应
    if ((cmd & 0x02) == 0) return 4;       // 非 expedited（段传输，此处按 4 处理）
    if ((cmd & 0x01) == 0) return 4;       // size 未指定，默认 4
    return static_cast<uint8_t>(4 - ((cmd >> 2) & 0x03));
}

// 组装 expedited SDO 下载（写）请求 payload（8 字节）
inline void packSdoDownload(uint16_t index, uint8_t subindex, uint32_t value,
                            uint8_t nbytes, uint8_t out[8]) {
    out[0] = sdoDownloadCmd(nbytes);
    out[1] = static_cast<uint8_t>(index & 0xFF);
    out[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    out[3] = subindex;
    out[4] = static_cast<uint8_t>(value & 0xFF);
    out[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
    out[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
    out[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

// 组装 SDO 上传（读）请求 payload（8 字节）
inline void packSdoUpload(uint16_t index, uint8_t subindex, uint8_t out[8]) {
    out[0] = SDO_UL_REQ;
    out[1] = static_cast<uint8_t>(index & 0xFF);
    out[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    out[3] = subindex;
    out[4] = out[5] = out[6] = out[7] = 0;
}

inline uint16_t sdoIndex(const uint8_t data[8]) {
    return static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
}
inline uint8_t sdoSubindex(const uint8_t data[8]) { return data[3]; }
inline uint32_t sdoData32(const uint8_t data[8]) {
    return static_cast<uint32_t>(data[4]) | (static_cast<uint32_t>(data[5]) << 8) |
           (static_cast<uint32_t>(data[6]) << 16) | (static_cast<uint32_t>(data[7]) << 24);
}

// ============================ 对象字典索引 (CiA-301 通信段) ============================
constexpr uint16_t OD_ERROR_REGISTER = 0x1001;
constexpr uint16_t OD_STORE_PARAMS = 0x1010;    // sub1=保存全部参数 ("save")
constexpr uint16_t OD_COB_ID_SYNC = 0x1005;
constexpr uint16_t OD_COMM_CYCLE_PERIOD = 0x1006;
constexpr uint16_t OD_PRODUCER_HEARTBEAT = 0x1017;
constexpr uint16_t OD_RPDO1_COMM = 0x1400;      // sub1=COB-ID, sub2=transmission type
constexpr uint16_t OD_RPDO2_COMM = 0x1401;
constexpr uint16_t OD_RPDO3_COMM = 0x1402;
constexpr uint16_t OD_RPDO4_COMM = 0x1403;
constexpr uint16_t OD_RPDO1_MAP = 0x1600;       // sub0=映射数, sub1..=映射项
constexpr uint16_t OD_RPDO2_MAP = 0x1601;
constexpr uint16_t OD_RPDO3_MAP = 0x1602;
constexpr uint16_t OD_RPDO4_MAP = 0x1603;
constexpr uint16_t OD_TPDO1_COMM = 0x1800;
constexpr uint16_t OD_TPDO2_COMM = 0x1801;
constexpr uint16_t OD_TPDO3_COMM = 0x1802;
constexpr uint16_t OD_TPDO4_COMM = 0x1803;
constexpr uint16_t OD_TPDO1_MAP = 0x1A00;
constexpr uint16_t OD_TPDO2_MAP = 0x1A01;
constexpr uint16_t OD_TPDO3_MAP = 0x1A02;
constexpr uint16_t OD_TPDO4_MAP = 0x1A03;

// PDO 通信参数子索引 & COB-ID 有效位（bit31=1 表示该 PDO 禁用）
constexpr uint8_t PDO_COMM_SUB_COBID = 1;
constexpr uint8_t PDO_COMM_SUB_TRANSMISSION = 2;
constexpr uint32_t PDO_COBID_INVALID_BIT = 0x80000000u;
// PDO 传输类型
constexpr uint8_t PDO_TRANS_SYNC_EVERY = 0x01;   // 每个 SYNC 触发（0x01..0xF0 = 每 N 个 SYNC）
constexpr uint8_t PDO_TRANS_ASYNC = 0xFF;        // 异步（厂商事件驱动，EDS 默认）

// ============================ 对象字典索引 (CiA-402 驱动规约) ============================
constexpr uint16_t OD_ERROR_CODE = 0x603F;         // U16 ro
constexpr uint16_t OD_CONTROLWORD = 0x6040;        // U16 rw  ← 状态机/运动触发
constexpr uint16_t OD_STATUSWORD = 0x6041;         // U16 ro  ← 状态机反馈
constexpr uint16_t OD_MODES_OF_OP = 0x6060;        // I8  rw  ← 运行模式设置
constexpr uint16_t OD_MODES_DISPLAY = 0x6061;      // I8  ro  ← 运行模式回显
constexpr uint16_t OD_POSITION_ACTUAL = 0x6064;    // I32 ro  (device counts)
constexpr uint16_t OD_VELOCITY_ACTUAL = 0x606C;    // I32 ro  (counts/s)
constexpr uint16_t OD_TARGET_TORQUE = 0x6071;      // I16 rw  (‰ 额定转矩)
constexpr uint16_t OD_TORQUE_ACTUAL = 0x6077;      // I16 ro  (‰ 额定转矩)
constexpr uint16_t OD_TARGET_POSITION = 0x607A;    // I32 rw  (counts)  PP/CSP
constexpr uint16_t OD_SW_POS_LIMIT = 0x607D;       // sub1=min sub2=max (counts)
constexpr uint16_t OD_MAX_PROFILE_VEL = 0x607F;    // U32 rw  (counts/s)
constexpr uint16_t OD_MAX_MOTOR_SPEED = 0x6080;    // U32 rw
constexpr uint16_t OD_PROFILE_VELOCITY = 0x6081;   // U32 rw  (counts/s)  PP
constexpr uint16_t OD_PROFILE_ACC = 0x6083;        // U32 rw
constexpr uint16_t OD_PROFILE_DEC = 0x6084;        // U32 rw
constexpr uint16_t OD_QUICKSTOP_DEC = 0x6085;      // U32 rw
constexpr uint16_t OD_HOMING_METHOD = 0x6098;      // I8  rw
constexpr uint16_t OD_HOMING_SPEEDS = 0x6099;      // sub1/sub2
constexpr uint16_t OD_HOMING_ACC = 0x609A;         // U32 rw
constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;    // I32 rw  (counts/s)  PV/CSV
constexpr uint16_t OD_SUPPORTED_DRIVE_MODES = 0x6502;  // U32 ro

// ============================ Modes of operation (0x6060/0x6061) ============================
enum class Cia402Mode : int8_t {
    NONE = 0,
    PP = 1,    // Profile Position（轮廓位置，点位）
    VL = 2,    // Velocity（速度）
    PV = 3,    // Profile Velocity（轮廓速度）
    TQ = 4,    // Profile Torque（轮廓力矩）
    HM = 6,    // Homing（回零）
    IP = 7,    // Interpolated Position（插补位置）
    CSP = 8,   // Cyclic Sync Position（循环同步位置，配合上位轨迹）
    CSV = 9,   // Cyclic Sync Velocity
    CST = 10,  // Cyclic Sync Torque
};

inline const char* modeName(Cia402Mode m) {
    switch (m) {
        case Cia402Mode::PP: return "PP(轮廓位置)";
        case Cia402Mode::VL: return "VL(速度)";
        case Cia402Mode::PV: return "PV(轮廓速度)";
        case Cia402Mode::TQ: return "TQ(轮廓力矩)";
        case Cia402Mode::HM: return "HM(回零)";
        case Cia402Mode::IP: return "IP(插补位置)";
        case Cia402Mode::CSP: return "CSP(循环同步位置)";
        case Cia402Mode::CSV: return "CSV(循环同步速度)";
        case Cia402Mode::CST: return "CST(循环同步力矩)";
        default: return "NONE";
    }
}

// ============================ Controlword (0x6040) ============================
// 状态机基础命令（低 4 位 + QuickStop/EnableVoltage 组合）
constexpr uint16_t CW_SHUTDOWN = 0x0006;          // -> Ready to switch on
constexpr uint16_t CW_SWITCH_ON = 0x0007;         // -> Switched on
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;  // -> Operation enabled
constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;   // -> Switch on disabled
constexpr uint16_t CW_QUICK_STOP = 0x0002;        // -> Quick stop active
constexpr uint16_t CW_DISABLE_OPERATION = 0x0007; // == Switch on
constexpr uint16_t CW_FAULT_RESET = 0x0080;       // bit7 上升沿清故障

// 运动相关 controlword 位
constexpr uint16_t CW_BIT_NEW_SETPOINT = 0x0010;        // bit4  PP: 新目标点触发（上升沿）
constexpr uint16_t CW_BIT_CHANGE_IMMEDIATELY = 0x0020;  // bit5  PP: 立即更新目标
constexpr uint16_t CW_BIT_ABS_REL = 0x0040;             // bit6  PP: 0=绝对 1=相对
constexpr uint16_t CW_BIT_HALT = 0x0100;                // bit8  暂停

// ============================ Statusword (0x6041) 状态机 ============================
enum class Cia402State {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown,
};

// 由 statusword 低位解析状态（CiA-402 标准位模式）
inline Cia402State parseState(uint16_t sw) {
    if ((sw & 0x004F) == 0x0000) return Cia402State::NotReadyToSwitchOn;
    if ((sw & 0x004F) == 0x0040) return Cia402State::SwitchOnDisabled;
    if ((sw & 0x006F) == 0x0021) return Cia402State::ReadyToSwitchOn;
    if ((sw & 0x006F) == 0x0023) return Cia402State::SwitchedOn;
    if ((sw & 0x006F) == 0x0027) return Cia402State::OperationEnabled;
    if ((sw & 0x006F) == 0x0007) return Cia402State::QuickStopActive;
    if ((sw & 0x004F) == 0x000F) return Cia402State::FaultReactionActive;
    if ((sw & 0x004F) == 0x0008) return Cia402State::Fault;
    return Cia402State::Unknown;
}

inline const char* stateName(Cia402State s) {
    switch (s) {
        case Cia402State::NotReadyToSwitchOn: return "未就绪(NotReadyToSwitchOn)";
        case Cia402State::SwitchOnDisabled: return "禁止上电(SwitchOnDisabled)";
        case Cia402State::ReadyToSwitchOn: return "就绪待上电(ReadyToSwitchOn)";
        case Cia402State::SwitchedOn: return "已上电(SwitchedOn)";
        case Cia402State::OperationEnabled: return "运行使能(OperationEnabled)";
        case Cia402State::QuickStopActive: return "快停(QuickStopActive)";
        case Cia402State::FaultReactionActive: return "故障处理中(FaultReactionActive)";
        case Cia402State::Fault: return "故障(Fault)";
        default: return "未知(Unknown)";
    }
}

// Statusword 常用状态位
constexpr uint16_t SW_BIT_VOLTAGE_ENABLED = 0x0010;   // bit4
constexpr uint16_t SW_BIT_WARNING = 0x0080;           // bit7
constexpr uint16_t SW_BIT_REMOTE = 0x0200;            // bit9
constexpr uint16_t SW_BIT_TARGET_REACHED = 0x0400;    // bit10 到位/达速
constexpr uint16_t SW_BIT_SETPOINT_ACK = 0x1000;      // bit12 PP: 目标已接收 / CSP: 跟随
constexpr uint16_t SW_BIT_FOLLOWING_ERROR = 0x2000;   // bit13

// ============================ 量纲换算 (device counts <-> SI) ============================
// ZeroErr eRob 110H 标准款：输出轴 19-bit 绝对编码器 = 524288 counts/圈，
//   齿轮比对象默认 1/1，0x6064/0x606C 直接按输出轴计数（手册 9.1，无需再乘减速比）。
//   高精度 HM/HS 款为 20-bit = 1048576 counts/圈。
constexpr double COUNTS_PER_REV_19BIT = 524288.0;
constexpr double COUNTS_PER_REV_20BIT = 1048576.0;
constexpr double TWO_PI = 6.283185307179586;

struct ScaleConfig {
    double counts_per_rev = COUNTS_PER_REV_19BIT;  // 输出轴每圈计数
    double sign = 1.0;                             // 关节正方向（+1/-1）
    double offset_rad = 0.0;                       // 关节零位偏置（SI 侧，counts=0 对应的关节角）

    double countsPerRad() const { return counts_per_rev / TWO_PI; }

    // 关节角(rad) -> 设备计数
    int32_t radToCounts(double rad) const {
        const double c = (sign * (rad - offset_rad)) * countsPerRad();
        return static_cast<int32_t>(std::lround(c));
    }
    // 设备计数 -> 关节角(rad)
    double countsToRad(int32_t counts) const {
        return sign * (static_cast<double>(counts) / countsPerRad()) + offset_rad;
    }
    // 角速度(rad/s) -> 计数/s（不含 offset，仅比例 + 方向）
    int32_t radsToCounts(double rad_s) const {
        return static_cast<int32_t>(std::lround(sign * rad_s * countsPerRad()));
    }
    double countsToRads(int32_t counts_s) const {
        return sign * (static_cast<double>(counts_s) / countsPerRad());
    }
};

}  // namespace canopen
}  // namespace elmue_can

#endif  // ELMUE_CAN__CANOPEN_PROTOCOL_HPP_
