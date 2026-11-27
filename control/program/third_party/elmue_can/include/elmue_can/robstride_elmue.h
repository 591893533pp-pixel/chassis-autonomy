// robstride_elmue.h —— 在 ElmueCan 上封装 Robstride 私有协议
//
// 公共 API 形状与 poke_hardware::RobstrideCanDriver 对齐，方便上层零改动切换后端：
//   Type 1  运控模式电机控制指令
//   Type 2  电机反馈（接收解析）
//   Type 3  使能电机
//   Type 4  失能电机
//   Type 6  设置机械零位
//   Type 17 读取参数（接收解析）
//   Type 18 写入参数
//   Type 21 故障反馈（接收解析）
//   Type 22 保存参数到 flash
//   Type 26 读取固件版本

#ifndef ELMUE_CAN__ROBSTRIDE_ELMUE_H_
#define ELMUE_CAN__ROBSTRIDE_ELMUE_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "elmue_can/elmue_can.h"
#include "elmue_can/robstride_protocol.hpp"

namespace elmue_can {

struct ParamReadResult {
    uint8_t motor_id = 0;
    uint16_t param_index = 0;
    float value = 0.0f;
    uint8_t raw_bytes[4]{};
    uint8_t status = 0;
    bool success = false;
    std::chrono::steady_clock::time_point timestamp{};
};

struct FirmwareVersion {
    uint8_t motor_id = 0;
    uint8_t version_bytes[5]{};
    std::string version_str;
    std::chrono::steady_clock::time_point timestamp{};
};

// 外来标准帧回调：把不属于 Robstride/达妙的标准帧（如 CANopen COB-ID）转发给上层。
// 用于与 ZeroErrCanopenDriver 共用同一条物理总线时的统一 RX 分发。
using ForeignFrameCallback =
    std::function<void(uint32_t can_id, const uint8_t *data, uint8_t dlc)>;

// 批量运控指令项（供 sendMotionControlBatch 一次性下发多电机）
struct MotionCommand {
    uint8_t motor_id = 0;
    MotorType motor_type = MotorType::RS00;
    double position = 0.0;
    double velocity = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double torque = 0.0;
};

/**
 * @brief Robstride 电机 ElmueCan 驱动
 *
 * 公共 API 形状与 poke_hardware::RobstrideCanDriver 对齐：
 *   - init() / close()
 *   - enableMotor / disableMotor / setZeroPosition
 *   - setRunMode / sendMotionControl
 *   - writeParameter / writeParameterInt / readParameter
 *   - saveParameters / queryFirmwareVersion
 *   - getMotorFeedback / getAllFeedbacks
 *   - setMotorType / setMotorTypes
 */
class RobstrideElmueDriver {
public:
    explicit RobstrideElmueDriver(uint8_t host_can_id = 0xFD,
                                  bool canfd_brs = true);
    ~RobstrideElmueDriver();

    RobstrideElmueDriver(const RobstrideElmueDriver &) = delete;
    RobstrideElmueDriver &operator=(const RobstrideElmueDriver &) = delete;

    bool init(const std::string &serial,
              uint32_t bitrate = 1'000'000,
              uint32_t dbitrate = 5'000'000,
              bool report_bus_errors = false);
    void close();
    bool isConnected() const { return can_.started(); }

    bool enableMotor(uint8_t motor_id);
    bool disableMotor(uint8_t motor_id, bool clear_fault = false);
    bool setZeroPosition(uint8_t motor_id);

    bool sendMotionControl(uint8_t motor_id,
                           MotorType motor_type,
                           double position,
                           double velocity,
                           double kp,
                           double kd,
                           double torque = 0.0);

    // 批量运控：把 count 个电机的运控帧通过 ElmueCan::sendFramesBatch 异步并行下发，
    // 大幅降低多电机单周期的 USB 累计耗时（用于 ros2_control 高频写循环）。
    bool sendMotionControlBatch(const MotionCommand *cmds, int count);

    // 只编码、不发送，供整机 SDK 将同一物理总线上的多协议帧统一调度。
    bool encodeMotionControlFrame(const MotionCommand &cmd, ElmueCanFrame &frame) const;
    BatchSendResult sendMotionControlBatchRealtime(const MotionCommand *cmds, int count,
                                                    uint32_t deadline_us = 1200);

    bool writeParameter(uint8_t motor_id, uint16_t param_index, float value);
    bool writeParameterInt(uint8_t motor_id, uint16_t param_index, uint32_t value);
    std::optional<ParamReadResult> readParameter(uint8_t motor_id,
                                                 uint16_t param_index,
                                                 double timeout_s = 0.5);

    bool setRunMode(uint8_t motor_id, RunMode mode);
    bool setPositionCSP(uint8_t motor_id, double position);
    bool setVelocityLimit(uint8_t motor_id, double velocity_limit);
    // Runtime-only RS02 communication type 24 active status reporting.
    // The motor defaults to a 10 ms report period; this call does not save
    // parameters to flash.
    bool setActiveReport(uint8_t motor_id, bool enabled);

    bool saveParameters(uint8_t motor_id);
    std::optional<FirmwareVersion> queryFirmwareVersion(uint8_t motor_id, double timeout_s = 0.5);

    MotorFeedback getMotorFeedback(uint8_t motor_id);
    std::map<uint8_t, MotorFeedback> getAllFeedbacks();
    uint32_t getFaultDetail(uint8_t motor_id);

    void setMotorType(uint8_t motor_id, MotorType type);
    void setMotorTypes(const std::map<uint8_t, MotorType> &types);

    // 达妙 DM 反馈 Master ID（MST_ID）映射：DM 反馈帧在独立标准帧 ID 上，
    // 需登记 motor_id -> recv_id 才能把反馈映射回逻辑电机。命令仍发到 motor_id（slave id）。
    void setMotorRecvId(uint8_t motor_id, uint32_t recv_id);
    void setMotorRecvIds(const std::map<uint8_t, uint32_t> &recv_ids);

    // 登记外来标准帧回调：RX 线程收到非 Robstride/非已登记达妙的标准帧时转发（如 CANopen）。
    // 用于共享总线时把 CANopen COB-ID 帧交给 ZeroErrCanopenDriver::handleRxFrame。
    void setForeignStdFrameCallback(ForeignFrameCallback cb);

    // 统计
    uint64_t getTxCount() const { return can_.txCount(); }
    uint64_t getRxUsbCount() const { return can_.rxUsbCount(); }
    uint64_t getRxMessageCount() const { return rx_message_count_.load(); }
    uint64_t getRxCanFrameCount() const { return rx_can_frame_count_.load(); }
    uint64_t getRxTxEchoCount() const { return rx_tx_echo_count_.load(); }
    uint64_t getRxCanErrorCount() const { return rx_can_error_count_.load(); }
    uint64_t getRxStringCount() const { return rx_string_count_.load(); }
    uint64_t getRxParamResponseCount() const { return rx_param_response_count_.load(); }
    uint32_t getLastCanErrorId() const { return last_can_error_id_.load(); }
    uint64_t getLastCanErrorData() const { return last_can_error_data_.load(); }
    uint64_t getTxErrors() const { return can_.txErrors(); }
    uint64_t getRxErrors() const { return can_.rxErrors(); }
    uint64_t getSendFailCount() const { return tx_fail_count_.load(); }
    uint64_t getSendRetryCount() const { return tx_retry_count_.load(); }

    const std::string &serial() const { return can_.serial(); }
    const std::string &lastError() const { return last_error_; }

    // 暴露底层 ElmueCan 供高阶用户使用（rs00_comm_test 压测）
    ElmueCan &elmue() { return can_; }

private:
    struct FeedbackSnapshotStore;
    bool sendRobstrideFrame(uint32_t can_id_29, const uint8_t data[8]);
    bool sendDamiaoFrame(uint32_t can_id_11, const uint8_t data[8]);  // 标准帧
    MotorType motorTypeFor(uint8_t motor_id);  // 查 motor_types_（默认 RS00）
    void onRxMessage(uint8_t msg_type, const uint8_t *payload, size_t len);
    void parseFeedbackFrame(uint32_t can_id, const uint8_t data[8]);
    void parseMitFeedbackFrame(uint32_t recv_id, const uint8_t data[8]);  // 达妙标准帧反馈
    void parseParamReadResponse(uint32_t can_id, const uint8_t data[8], uint8_t motor_id);
    void parseFaultFrame(const uint8_t data[8], uint8_t motor_id);
    void parseVersionResponse(const uint8_t data[8], uint8_t motor_id);
    void publishFeedbackSnapshot(const MotorFeedback &feedback);

    ElmueCan can_;
    uint8_t host_can_id_;
    bool canfd_brs_;
    bool fd_active_ = false;
    std::string last_error_;

    std::mutex feedback_mutex_;
    std::map<uint8_t, MotorFeedback> feedbacks_;
    std::map<uint8_t, MotorType> motor_types_;
    std::map<uint8_t, uint32_t> motor_recv_id_;   // motor_id -> DM 反馈 MST_ID
    std::map<uint32_t, uint8_t> recv_id_to_motor_; // DM 反馈 MST_ID -> motor_id
    std::map<uint8_t, ParamReadResult> param_results_;
    std::map<uint8_t, FirmwareVersion> firmware_versions_;
    std::map<uint8_t, uint32_t> fault_details_;
    std::map<uint8_t, bool> awaiting_version_;

    std::atomic<uint64_t> tx_fail_count_{0};
    std::atomic<uint64_t> tx_retry_count_{0};
    std::atomic<uint64_t> rx_message_count_{0};
    std::atomic<uint64_t> rx_can_frame_count_{0};
    std::atomic<uint64_t> rx_tx_echo_count_{0};
    std::atomic<uint64_t> rx_can_error_count_{0};
    std::atomic<uint64_t> rx_string_count_{0};
    std::atomic<uint64_t> rx_param_response_count_{0};
    std::atomic<uint32_t> last_can_error_id_{0};
    std::atomic<uint64_t> last_can_error_data_{0};

    ForeignFrameCallback foreign_std_cb_;  // 共享总线：转发非本协议标准帧（CANopen 等）
};

}  // namespace elmue_can

#endif  // ELMUE_CAN__ROBSTRIDE_ELMUE_H_
