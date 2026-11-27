// zeroerr_canopen.h —— 在 ElmueCan 上实现 CANopen (CiA-301) + CiA-402 主站，
//   驱动 ZeroErr eDriver / eRob 系列谐波执行器（如 POKE 腰俯仰 eRob 110H, node 43）。
//
// 设计目标：与 elmue_can::RobstrideElmueDriver 同栈共存，可在同一条经典 CAN 1M 总线上
// 与 Robstride（扩展帧 29 位）/ 达妙（标准帧 MIT）共线——CANopen 用标准帧 COB-ID
// (0x000~0x77F)，与二者天然按 CAN ID 区分。
//
// 两种运行模式：
//   1) 独占模式 init(serial, bitrate)：本类自持一个 ElmueCan，独占该 CANable 适配器。
//      用于独立自检 / id43 单独一条总线。
//   2) 共享模式 attach(ElmueCan*)：不拥有适配器，仅发送 + 处理被转发来的 RX 帧。
//      用于与 RobstrideElmueDriver 共用同一个 libusb 句柄（同一物理总线）。此时上层
//      需在其统一 RX 回调中，对 CANopen COB-ID 的标准帧调用 handleRxFrame()。
//
// 控制能力：NMT 启停、expedited SDO 读写对象字典、CiA-402 状态机（使能/失能/清故障/快停）、
//   运行模式切换、PP 点位运动、PV 速度、CSP/CSV 周期控制（PDO）、TPDO 反馈解析、量纲换算。

#ifndef ELMUE_CAN__ZEROERR_CANOPEN_H_
#define ELMUE_CAN__ZEROERR_CANOPEN_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "elmue_can/canopen_protocol.hpp"
#include "elmue_can/elmue_can.h"

namespace elmue_can {

// 单节点运动/状态反馈（SI 单位；由 SDO 读或 TPDO 解析填充）
struct CanopenFeedback {
    uint8_t node_id = 0;
    double position = 0.0;      // rad（已按 ScaleConfig 换算，含方向/偏置）
    double velocity = 0.0;      // rad/s
    double torque = 0.0;        // ‰ 额定转矩（0x6077 原值；如需 Nm 需乘额定转矩）
    uint16_t statusword = 0;
    canopen::Cia402State state = canopen::Cia402State::Unknown;
    int8_t mode_display = 0;    // 0x6061 回显
    uint16_t error_code = 0;    // 0x603F
    uint8_t nmt_state = 0;      // 心跳最近上报的 NMT 状态
    bool online = false;        // 收到过任一有效帧
    bool position_valid = false;// 已收到过 TPDO2(0x6064) 位置（避免仅心跳时误用 position=0 突跳）
    bool target_reached = false;// statusword bit10
    std::chrono::steady_clock::time_point last_update{};
};

// SDO 单次传输结果
struct SdoResult {
    bool success = false;
    bool aborted = false;
    uint32_t value = 0;       // 上传时的读回值（≤4 字节）
    uint32_t abort_code = 0;  // aborted 时的中止码
};

/**
 * @brief ZeroErr / 通用 CiA-402 CANopen 主站驱动（基于 ElmueCan）
 */
class ZeroErrCanopenDriver {
public:
    explicit ZeroErrCanopenDriver(uint8_t master_node_id = 0x7D /*253*/);
    ~ZeroErrCanopenDriver();

    ZeroErrCanopenDriver(const ZeroErrCanopenDriver&) = delete;
    ZeroErrCanopenDriver& operator=(const ZeroErrCanopenDriver&) = delete;

    // ---- 生命周期 ----
    // 独占模式：自持并打开一个 CANable（经典 CAN，ZeroErr 仅支持 1 Mbps）。
    bool init(const std::string& serial, uint32_t bitrate = 1'000'000);
    // 共享模式：附着到已由他人打开/启动的 ElmueCan（不拥有、不 close 它）。
    void attach(ElmueCan* shared_can);
    void close();
    bool isConnected() const;

    // 供上层统一 RX 回调分发（共享模式）：仅处理 CANopen 标准帧（COB-ID 0x000~0x77F）。
    // 独占模式下由内部 RX 回调自动调用，无需外部再调。
    void handleRxFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);

    // ---- 节点登记 ----
    void addNode(uint8_t node_id, const canopen::ScaleConfig& scale = {});
    void setScale(uint8_t node_id, const canopen::ScaleConfig& scale);
    canopen::ScaleConfig getScale(uint8_t node_id) const;

    // ---- NMT 网络管理 ----
    bool nmtStart(uint8_t node_id);       // -> Operational（PDO 生效）
    bool nmtStop(uint8_t node_id);
    bool nmtEnterPreOp(uint8_t node_id);
    bool nmtResetNode(uint8_t node_id);
    bool nmtResetComm(uint8_t node_id);

    // ---- SDO（expedited，阻塞等待响应）----
    SdoResult sdoWrite(uint8_t node_id, uint16_t index, uint8_t sub,
                       uint32_t value, uint8_t nbytes, double timeout_s = 0.5);
    SdoResult sdoRead(uint8_t node_id, uint16_t index, uint8_t sub,
                      double timeout_s = 0.5);

    // ---- CiA-402 状态机 ----
    // 读当前 statusword 并解析状态（走 SDO）
    std::optional<uint16_t> readStatusword(uint8_t node_id, double timeout_s = 0.5);
    canopen::Cia402State readState(uint8_t node_id, double timeout_s = 0.5);
    bool setControlword(uint8_t node_id, uint16_t controlword);
    // 驱动状态机至 Operation enabled（自动 Shutdown->SwitchOn->EnableOp，遇 Fault 先清障）
    bool enable(uint8_t node_id, double timeout_s = 2.0);
    // 失能（Disable voltage -> Switch on disabled）
    bool disable(uint8_t node_id);
    bool faultReset(uint8_t node_id, double timeout_s = 1.0);
    bool quickStop(uint8_t node_id);

    // ---- 运行模式 ----
    bool setMode(uint8_t node_id, canopen::Cia402Mode mode, double timeout_s = 0.5);
    std::optional<canopen::Cia402Mode> readMode(uint8_t node_id, double timeout_s = 0.5);

    // ---- 运动指令 ----
    // PP 轮廓位置点位：写 profile velocity（可选）+ target position + new-setpoint 脉冲。
    //   immediate=true 立即更新目标（bit5），relative=true 相对当前（bit6）。
    bool moveToPositionPP(uint8_t node_id, double position_rad,
                          double profile_velocity_rads = 0.0,
                          bool immediate = true, bool relative = false);
    // Stop an active PP trajectory without disabling the drive. The next PP
    // setpoint clears HALT as part of its normal enable-operation controlword.
    bool haltPositionPP(uint8_t node_id);
    // 设置 PP 的轮廓速度/加减速（counts/s；<=0 则跳过）
    bool setProfileParams(uint8_t node_id, double profile_velocity_rads,
                          double profile_acc_rads2 = 0.0, double profile_dec_rads2 = 0.0);
    // CSP 周期位置（需 mode=CSP 且已使能）：经 RPDO2(6040+607A) 下发一帧，controlword=EnableOp。
    bool setTargetPositionCSP(uint8_t node_id, double position_rad);
    // 只编码、不发送；RPDO 必须先于 SYNC 下发。
    bool encodeTargetPositionCSP(uint8_t node_id, double position_rad, ElmueCanFrame& frame);
    static ElmueCanFrame encodeSyncFrame();
    // PV/CSV 速度目标（counts/s）
    bool setTargetVelocity(uint8_t node_id, double velocity_rads);
    // 通过 SDO 直接读当前位置（counts->rad），不依赖 PDO
    std::optional<double> readPosition(uint8_t node_id, double timeout_s = 0.5);

    // ---- PDO ----
    // 广播 SYNC（0x080, dlc=0）；用于同步 PDO 触发。
    bool sendSync();
    // 按 EDS 默认映射启用常用 PDO：使能 TPDO2(6041+6064)/TPDO3(6041+606C) 每 SYNC 发送，
    //   RPDO2(6040+607A) 每 SYNC 生效。返回是否全部 SDO 配置成功。
    bool configureDefaultPdos(uint8_t node_id, uint8_t sync_every = 1, double timeout_s = 0.5);

    // ---- 反馈 ----
    CanopenFeedback getFeedback(uint8_t node_id);
    std::map<uint8_t, CanopenFeedback> getAllFeedbacks();

    const std::string& lastError() const { return last_error_; }
    ElmueCan* elmue() { return can_; }

private:
    struct FeedbackSnapshotStore;
    bool sendCanopenFrame(uint32_t cob_id, const uint8_t* data, uint8_t dlc);
    bool nmtCommand(uint8_t command, uint8_t node_id);
    // 阻塞式 SDO 收发：清响应槽 -> 发请求 -> 等 0x580+id 响应/超时。
    SdoResult sdoTransfer(uint8_t node_id, const uint8_t req[8], double timeout_s);
    // RX 分帧处理
    void onSdoResponse(uint8_t node_id, const uint8_t data[8]);
    void onTpdo(uint32_t func_base, uint8_t node_id, const uint8_t* data, uint8_t dlc);
    void onHeartbeat(uint8_t node_id, const uint8_t* data, uint8_t dlc);
    void onEmcy(uint8_t node_id, const uint8_t* data, uint8_t dlc);
    canopen::ScaleConfig scaleFor(uint8_t node_id);
    void publishFeedbackSnapshot(const CanopenFeedback& feedback);

    ElmueCan* can_ = nullptr;            // 当前使用的句柄（独占->owned_can_；共享->外部）
    std::unique_ptr<ElmueCan> owned_can_;// 独占模式所有权
    bool shared_ = false;
    uint8_t master_node_id_;
    std::string last_error_;

    mutable std::mutex state_mutex_;
    std::map<uint8_t, canopen::ScaleConfig> scales_;
    std::map<uint8_t, CanopenFeedback> feedbacks_;

    // SDO 响应同步
    std::mutex sdo_mutex_;
    std::condition_variable sdo_cv_;
    struct SdoSlot {
        bool valid = false;
        uint8_t cmd = 0;
        uint16_t index = 0;
        uint8_t sub = 0;
        uint32_t data = 0;
    };
    std::map<uint8_t, SdoSlot> sdo_slots_;
};

}  // namespace elmue_can

#endif  // ELMUE_CAN__ZEROERR_CANOPEN_H_
