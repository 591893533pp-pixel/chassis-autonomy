// robstride_elmue.cpp —— Robstride 私有协议在 ElmueCan 上的实现

#include "elmue_can/robstride_elmue.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <new>
#include <thread>

namespace elmue_can {

struct RobstrideElmueDriver::FeedbackSnapshotStore {
    struct Slot {
        std::atomic<uint32_t> sequence{0};
        std::atomic<double> position{0.0};
        std::atomic<double> velocity{0.0};
        std::atomic<double> torque{0.0};
        std::atomic<double> temperature{0.0};
        std::atomic<int64_t> update_ns{0};
        std::atomic<uint32_t> meta{0};
    };
    std::array<Slot, 256> slots{};
};

namespace {
class FeedbackRegistry {
public:
    FeedbackRegistry() {
        for (auto& key : keys_) key.store(nullptr, std::memory_order_relaxed);
        for (auto& value : values_) value.store(nullptr, std::memory_order_relaxed);
    }
    void* get(const void* key) const {
        for (std::size_t i = 0; i < keys_.size(); ++i)
            if (keys_[i].load(std::memory_order_acquire) == key)
                return values_[i].load(std::memory_order_acquire);
        return nullptr;
    }
    bool add(const void* key, void* value) {
        for (std::size_t i = 0; i < keys_.size(); ++i) {
            const void* expected = nullptr;
            if (keys_[i].compare_exchange_strong(expected, key, std::memory_order_acq_rel)) {
                values_[i].store(value, std::memory_order_release); return true;
            }
        }
        return false;
    }
    void* remove(const void* key) {
        for (std::size_t i = 0; i < keys_.size(); ++i) if (keys_[i].load() == key) {
            void* value = values_[i].exchange(nullptr, std::memory_order_acq_rel);
            keys_[i].store(nullptr, std::memory_order_release); return value;
        }
        return nullptr;
    }
private:
    std::array<std::atomic<const void*>, 64> keys_{};
    std::array<std::atomic<void*>, 64> values_{};
};
FeedbackRegistry& feedbackRegistry() { static FeedbackRegistry registry; return registry; }
}  // namespace

RobstrideElmueDriver::RobstrideElmueDriver(uint8_t host_can_id, bool canfd_brs)
    : host_can_id_(host_can_id), canfd_brs_(canfd_brs) {
    auto* store = new (std::nothrow) FeedbackSnapshotStore();
    if (!store || !feedbackRegistry().add(this, store)) {
        delete store; last_error_ = "feedback snapshot registry exhausted";
    }
}

RobstrideElmueDriver::~RobstrideElmueDriver() {
    close();
    delete static_cast<FeedbackSnapshotStore*>(feedbackRegistry().remove(this));
}

bool RobstrideElmueDriver::init(const std::string &serial, uint32_t bitrate, uint32_t dbitrate,
                                bool report_bus_errors) {
    if (!can_.open(serial)) {
        last_error_ = can_.lastError();
        return false;
    }

    // 仲裁段采样点 0.75（内核 gs_usb 默认值）：达妙 CAN FD 反馈帧在 0.80 下会接收失败
    if (!can_.setBitrateHz(bitrate, 0.75)) {
        last_error_ = can_.lastError();
        can_.close();
        return false;
    }

    fd_active_ = false;
    if (dbitrate > 0 && can_.supportsFd()) {
        if (can_.setBitrateFDHz(dbitrate, 0.75)) {
            fd_active_ = true;
        }
    }

    can_.setRxCallback([this](uint8_t msg_type, const uint8_t *payload, size_t len) {
        onRxMessage(msg_type, payload, len);
    });

    const uint32_t flags = report_bus_errors ? GS_DevFlagBerrReporting : 0;
    if (!can_.start(flags, true)) {
        last_error_ = can_.lastError();
        can_.close();
        return false;
    }

    return true;
}

void RobstrideElmueDriver::close() { can_.close(); }

bool RobstrideElmueDriver::sendRobstrideFrame(uint32_t can_id_29, const uint8_t data[8]) {
    ElmueCanFrame frame{};
    frame.can_id = can_id_29 & CAN_MASK_29;
    frame.extended = true;
    frame.fd = fd_active_;
    frame.brs = fd_active_ && canfd_brs_;
    frame.data_len = 8;
    std::memcpy(frame.data, data, 8);

    constexpr int kMaxRetries = 2;
    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        if (can_.sendFrame(frame, 0)) {
            return true;
        }
        if (attempt < kMaxRetries) {
            tx_retry_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    tx_fail_count_.fetch_add(1, std::memory_order_relaxed);
    last_error_ = can_.lastError();
    return false;
}

bool RobstrideElmueDriver::sendDamiaoFrame(uint32_t can_id_11, const uint8_t data[8]) {
    ElmueCanFrame frame{};
    frame.can_id = can_id_11 & CAN_MASK_11;
    frame.extended = false;  // 达妙 MIT 走标准帧（11 位 ID）
    frame.fd = fd_active_;
    frame.brs = fd_active_ && canfd_brs_;
    frame.data_len = 8;
    std::memcpy(frame.data, data, 8);

    constexpr int kMaxRetries = 2;
    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        if (can_.sendFrame(frame, 0)) {
            return true;
        }
        if (attempt < kMaxRetries) {
            tx_retry_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    tx_fail_count_.fetch_add(1, std::memory_order_relaxed);
    last_error_ = can_.lastError();
    return false;
}

MotorType RobstrideElmueDriver::motorTypeFor(uint8_t motor_id) {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    auto it = motor_types_.find(motor_id);
    return it != motor_types_.end() ? it->second : MotorType::RS00;
}

bool RobstrideElmueDriver::enableMotor(uint8_t motor_id) {
    if (protocolFamily(motorTypeFor(motor_id)) == ProtocolFamily::DAMIAO_MIT) {
        const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, damiao::CMD_ENABLE};
        return sendDamiaoFrame(motor_id, data);
    }
    const uint32_t can_id = buildExtendedCanId(3, host_can_id_, motor_id);
    uint8_t data[8] = {0};
    return sendRobstrideFrame(can_id, data);
}

bool RobstrideElmueDriver::disableMotor(uint8_t motor_id, bool clear_fault) {
    if (protocolFamily(motorTypeFor(motor_id)) == ProtocolFamily::DAMIAO_MIT) {
        // 达妙失能即停止运行；清故障由重新使能完成（MIT 无独立清障字节）
        const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, damiao::CMD_DISABLE};
        return sendDamiaoFrame(motor_id, data);
    }
    const uint32_t can_id = buildExtendedCanId(4, host_can_id_, motor_id);
    uint8_t data[8] = {0};
    if (clear_fault) data[0] = 1;
    return sendRobstrideFrame(can_id, data);
}

bool RobstrideElmueDriver::setZeroPosition(uint8_t motor_id) {
    if (protocolFamily(motorTypeFor(motor_id)) == ProtocolFamily::DAMIAO_MIT) {
        const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, damiao::CMD_SET_ZERO};
        return sendDamiaoFrame(motor_id, data);
    }
    const uint32_t can_id = buildExtendedCanId(6, host_can_id_, motor_id);
    uint8_t data[8] = {0};
    data[0] = 1;
    return sendRobstrideFrame(can_id, data);
}

bool RobstrideElmueDriver::sendMotionControl(uint8_t motor_id,
                                             MotorType motor_type,
                                             double position,
                                             double velocity,
                                             double kp,
                                             double kd,
                                             double torque) {
    if (protocolFamily(motor_type) == ProtocolFamily::DAMIAO_MIT) {
        uint8_t data[8];
        packMitControlPayload(motor_type, position, velocity, kp, kd, torque, data);
        return sendDamiaoFrame(motor_id, data);
    }
    uint8_t data[8];
    const uint16_t torque_raw =
        packMotionControlPayload(motor_type, position, velocity, kp, kd, torque, data);
    const uint32_t can_id = buildExtendedCanId(1, torque_raw, motor_id);
    return sendRobstrideFrame(can_id, data);
}

bool RobstrideElmueDriver::sendMotionControlBatch(const MotionCommand *cmds, int count) {
    return sendMotionControlBatchRealtime(cmds, count, 4000).success();
}

bool RobstrideElmueDriver::encodeMotionControlFrame(const MotionCommand &cmd,
                                                     ElmueCanFrame &f) const {
    uint8_t data[8];
    if (protocolFamily(cmd.motor_type) == ProtocolFamily::DAMIAO_MIT) {
        packMitControlPayload(cmd.motor_type, cmd.position, cmd.velocity,
                              cmd.kp, cmd.kd, cmd.torque, data);
        f.can_id = cmd.motor_id & CAN_MASK_11;
        f.extended = false;
    } else {
        const uint16_t torque_raw = packMotionControlPayload(
            cmd.motor_type, cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque, data);
        f.can_id = buildExtendedCanId(1, torque_raw, cmd.motor_id) & CAN_MASK_29;
        f.extended = true;
    }
    f.fd = fd_active_;
    f.brs = fd_active_ && canfd_brs_;
    f.data_len = 8;
    std::memcpy(f.data, data, 8);
    return true;
}

BatchSendResult RobstrideElmueDriver::sendMotionControlBatchRealtime(
    const MotionCommand *cmds, int count, uint32_t deadline_us) {
    if (!cmds || count <= 0) {
        BatchSendResult r;
        r.errors = 1;
        return r;
    }
    constexpr int kMax = 32;
    if (count > kMax) {
        count = kMax;
    }

    std::array<ElmueCanFrame, kMax> frames{};
    for (int i = 0; i < count; ++i) {
        encodeMotionControlFrame(cmds[i], frames[i]);
    }

    auto result = can_.sendFramesBatchRealtime(frames.data(), count, deadline_us);
    if (!result.success()) {
        tx_fail_count_.fetch_add(1, std::memory_order_relaxed);
        last_error_ = can_.lastError();
    }
    return result;
}

bool RobstrideElmueDriver::writeParameter(uint8_t motor_id, uint16_t param_index, float value) {
    const uint32_t can_id = buildExtendedCanId(18, host_can_id_, motor_id);
    uint8_t data[8] = {0};
    data[0] = param_index & 0xFF;
    data[1] = (param_index >> 8) & 0xFF;
    data[2] = 0;
    data[3] = 0;
    uint32_t value_raw = 0;
    std::memcpy(&value_raw, &value, sizeof(float));
    data[4] = value_raw & 0xFF;
    data[5] = (value_raw >> 8) & 0xFF;
    data[6] = (value_raw >> 16) & 0xFF;
    data[7] = (value_raw >> 24) & 0xFF;
    return sendRobstrideFrame(can_id, data);
}

bool RobstrideElmueDriver::writeParameterInt(uint8_t motor_id, uint16_t param_index, uint32_t value) {
    const uint32_t can_id = buildExtendedCanId(18, host_can_id_, motor_id);
    uint8_t data[8] = {0};
    data[0] = param_index & 0xFF;
    data[1] = (param_index >> 8) & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = value & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = (value >> 16) & 0xFF;
    data[7] = (value >> 24) & 0xFF;
    return sendRobstrideFrame(can_id, data);
}

std::optional<ParamReadResult> RobstrideElmueDriver::readParameter(uint8_t motor_id,
                                                                   uint16_t param_index,
                                                                   double timeout_s) {
    {
        std::lock_guard<std::mutex> lk(feedback_mutex_);
        param_results_.erase(motor_id);
    }

    const uint32_t can_id = buildExtendedCanId(17, host_can_id_, motor_id);
    uint8_t data[8] = {0};
    data[0] = param_index & 0xFF;
    data[1] = (param_index >> 8) & 0xFF;
    if (!sendRobstrideFrame(can_id, data)) {
        return std::nullopt;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::microseconds(static_cast<int64_t>(timeout_s * 1e6));
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lk(feedback_mutex_);
            auto it = param_results_.find(motor_id);
            if (it != param_results_.end() && it->second.param_index == param_index) {
                return it->second;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    return std::nullopt;
}

bool RobstrideElmueDriver::setRunMode(uint8_t motor_id, RunMode mode) {
    return writeParameterInt(motor_id, robstride::PARAM_RUN_MODE, static_cast<uint32_t>(mode));
}

bool RobstrideElmueDriver::setPositionCSP(uint8_t motor_id, double position) {
    return writeParameter(motor_id, robstride::PARAM_LOC_REF, static_cast<float>(position));
}

bool RobstrideElmueDriver::setVelocityLimit(uint8_t motor_id, double velocity_limit) {
    return writeParameter(motor_id, robstride::PARAM_LIMIT_SPD, static_cast<float>(velocity_limit));
}

bool RobstrideElmueDriver::setActiveReport(uint8_t motor_id, bool enabled) {
    const uint32_t can_id = buildExtendedCanId(24, host_can_id_, motor_id);
    const uint8_t data[8] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
        static_cast<uint8_t>(enabled ? 0x01 : 0x00), 0x00};
    return sendRobstrideFrame(can_id, data);
}

bool RobstrideElmueDriver::saveParameters(uint8_t motor_id) {
    const uint32_t can_id = buildExtendedCanId(22, host_can_id_, motor_id);
    const uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    return sendRobstrideFrame(can_id, data);
}

std::optional<FirmwareVersion> RobstrideElmueDriver::queryFirmwareVersion(uint8_t motor_id, double timeout_s) {
    {
        std::lock_guard<std::mutex> lk(feedback_mutex_);
        firmware_versions_.erase(motor_id);
        awaiting_version_[motor_id] = true;
    }

    const uint32_t can_id = buildExtendedCanId(26, host_can_id_, motor_id);
    uint8_t data[8] = {0x00, 0xC4, 0, 0, 0, 0, 0, 0};
    if (!sendRobstrideFrame(can_id, data)) {
        return std::nullopt;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::microseconds(static_cast<int64_t>(timeout_s * 1e6));
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lk(feedback_mutex_);
            auto it = firmware_versions_.find(motor_id);
            if (it != firmware_versions_.end()) {
                return it->second;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    return std::nullopt;
}

MotorFeedback RobstrideElmueDriver::getMotorFeedback(uint8_t motor_id) {
    // Copy a complete sample under the same mutex used by the RX writer.
    // Exhausting a seqlock retry budget is not evidence of a missing CAN frame.
    // Preserve the RX timestamp: callers still enforce their freshness limit.
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    const auto it = feedbacks_.find(motor_id);
    return it == feedbacks_.end() ? MotorFeedback{} : it->second;
}

void RobstrideElmueDriver::publishFeedbackSnapshot(const MotorFeedback &fb) {
    auto* snapshots = static_cast<FeedbackSnapshotStore*>(feedbackRegistry().get(this));
    if (!snapshots) return;
    auto& slot = snapshots->slots[fb.motor_id];
    slot.sequence.fetch_add(1, std::memory_order_acq_rel);
    slot.position.store(fb.position, std::memory_order_relaxed);
    slot.velocity.store(fb.velocity, std::memory_order_relaxed);
    slot.torque.store(fb.torque, std::memory_order_relaxed);
    slot.temperature.store(fb.temperature, std::memory_order_relaxed);
    slot.update_ns.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        fb.last_update.time_since_epoch()).count(), std::memory_order_relaxed);
    const uint32_t meta = (fb.is_valid ? 1u : 0u) |
        (static_cast<uint32_t>(fb.mode_state) << 8) |
        (static_cast<uint32_t>(fb.fault_code) << 16);
    slot.meta.store(meta, std::memory_order_release);
    slot.sequence.fetch_add(1, std::memory_order_release);
}

std::map<uint8_t, MotorFeedback> RobstrideElmueDriver::getAllFeedbacks() {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    return feedbacks_;
}

uint32_t RobstrideElmueDriver::getFaultDetail(uint8_t motor_id) {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    auto it = fault_details_.find(motor_id);
    return it != fault_details_.end() ? it->second : 0u;
}

void RobstrideElmueDriver::setMotorType(uint8_t motor_id, MotorType type) {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    motor_types_[motor_id] = type;
}

void RobstrideElmueDriver::setMotorTypes(const std::map<uint8_t, MotorType> &types) {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    motor_types_ = types;
}

void RobstrideElmueDriver::setMotorRecvId(uint8_t motor_id, uint32_t recv_id) {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    motor_recv_id_[motor_id] = recv_id;
    recv_id_to_motor_[recv_id] = motor_id;
}

void RobstrideElmueDriver::setMotorRecvIds(const std::map<uint8_t, uint32_t> &recv_ids) {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    motor_recv_id_ = recv_ids;
    recv_id_to_motor_.clear();
    for (const auto &kv : recv_ids) {
        recv_id_to_motor_[kv.second] = kv.first;
    }
}

void RobstrideElmueDriver::setForeignStdFrameCallback(ForeignFrameCallback cb) {
    foreign_std_cb_ = std::move(cb);
}

void RobstrideElmueDriver::onRxMessage(uint8_t msg_type, const uint8_t *payload, size_t len) {
    rx_message_count_.fetch_add(1, std::memory_order_relaxed);
    if (msg_type == MSG_TxEcho) {
        rx_tx_echo_count_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (msg_type == MSG_Error) {
        rx_can_error_count_.fetch_add(1, std::memory_order_relaxed);
        constexpr size_t kErrorWithoutTimestamp = offsetof(kErrorElmue, timestamp);
        if (len >= kErrorWithoutTimestamp) {
            const auto *error = reinterpret_cast<const kErrorElmue *>(payload);
            uint64_t data = 0;
            std::memcpy(&data, error->err_data, sizeof(data));
            last_can_error_id_.store(error->err_id, std::memory_order_relaxed);
            last_can_error_data_.store(data, std::memory_order_relaxed);
        }
        return;
    }
    if (msg_type == MSG_String) {
        rx_string_count_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (msg_type != MSG_RxFrame) return;
    rx_can_frame_count_.fetch_add(1, std::memory_order_relaxed);

    // RX 帧布局: header(2) + flags(1) + can_id(4) [+ timestamp(4)] + data。
    // 注意：本固件在未开启 GS_DevFlagTimestamp 时【不发送】4 字节时间戳，
    //   实测帧长 = 7(头部) + 8(数据) = 15，而 sizeof(kRxFrameElmue)=11 含时间戳。
    //   Robstride/达妙 帧恒为 8 字节数据，故按"CAN 数据位于帧末尾 8 字节"取偏移；
    //   CANopen 帧为变长（心跳 1B、TPDO 6B 等），须按 payload+kCanIdEnd/len-kCanIdEnd 取。
    constexpr size_t kCanIdEnd = offsetof(kRxFrameElmue, timestamp);  // = 7
    if (len < kCanIdEnd) return;  // 至少需 can_id（4B，落在 offset 3..6）
    const auto *rx = reinterpret_cast<const kRxFrameElmue *>(payload);
    const bool is_extended = (rx->can_id & CAN_ID_29Bit) != 0;

    // 标准帧（11 位）：达妙 MIT 反馈走独立 MST_ID(recv_id)；其余（CANopen 等）转发外来钩子。
    // CANopen 变长，须在 8 字节守卫之前处理，否则短帧（心跳/TPDO）会被丢弃。
    if (!is_extended) {
        const uint32_t recv_id = rx->can_id & CAN_MASK_11;
        bool known = false;
        {
            std::lock_guard<std::mutex> lk(feedback_mutex_);
            known = recv_id_to_motor_.count(recv_id) > 0;
        }
        if (known) {
            if (len >= kCanIdEnd + 8) parseMitFeedbackFrame(recv_id, payload + (len - 8));
        } else if (foreign_std_cb_) {
            const uint8_t dlc = static_cast<uint8_t>(len - kCanIdEnd);
            foreign_std_cb_(recv_id, payload + kCanIdEnd, dlc);
        }
        return;
    }

    // 扩展帧（29 位）：Robstride 私有协议，恒为 8 字节数据
    if (len < kCanIdEnd + 8) return;
    const uint8_t *data = payload + (len - 8);
    const uint32_t can_id = rx->can_id & CAN_MASK_29;
    const uint8_t comm_type = (can_id >> 24) & 0x1F;
    const uint8_t motor_id = (can_id >> 8) & 0xFF;

    // Type 2 is command-response feedback. Firmware >=0.2.3.0 may emit the
    // same payload as communication type 24 when active reporting is enabled.
    if (comm_type == 2 || comm_type == 24) {
        bool awaiting_ver = false;
        {
            std::lock_guard<std::mutex> lk(feedback_mutex_);
            auto it = awaiting_version_.find(motor_id);
            if (it != awaiting_version_.end() && it->second) {
                it->second = false;
                awaiting_ver = true;
            }
        }
        if (awaiting_ver) {
            parseVersionResponse(data, motor_id);
        } else {
            parseFeedbackFrame(can_id, data);
        }
    } else if (comm_type == 17) {
        rx_param_response_count_.fetch_add(1, std::memory_order_relaxed);
        parseParamReadResponse(can_id, data, motor_id);
    } else if (comm_type == 21) {
        parseFaultFrame(data, motor_id);
    }
}

void RobstrideElmueDriver::parseMitFeedbackFrame(uint32_t recv_id, const uint8_t data[8]) {
    uint8_t motor_id = 0;
    MotorType mtype = MotorType::RS00;
    {
        std::lock_guard<std::mutex> lk(feedback_mutex_);
        auto it = recv_id_to_motor_.find(recv_id);
        if (it == recv_id_to_motor_.end()) return;
        motor_id = it->second;
        auto tit = motor_types_.find(motor_id);
        if (tit != motor_types_.end()) mtype = tit->second;
    }

    MotorFeedback fb = parseMitFeedbackPayload(data, mtype, motor_id);
    publishFeedbackSnapshot(fb);
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    feedbacks_[motor_id] = fb;
}

void RobstrideElmueDriver::parseFeedbackFrame(uint32_t can_id, const uint8_t data[8]) {
    const uint8_t motor_id = (can_id >> 8) & 0xFF;
    if (motor_id == 0) return;

    MotorType mtype = MotorType::RS00;
    {
        std::lock_guard<std::mutex> lk(feedback_mutex_);
        auto it = motor_types_.find(motor_id);
        if (it != motor_types_.end()) mtype = it->second;
    }

    MotorFeedback fb = parseFeedbackPayload(can_id, data, mtype);
    publishFeedbackSnapshot(fb);
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    feedbacks_[motor_id] = fb;
}

void RobstrideElmueDriver::parseParamReadResponse(uint32_t can_id, const uint8_t data[8], uint8_t motor_id) {
    ParamReadResult result{};
    result.motor_id = motor_id;
    result.param_index = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    std::memcpy(result.raw_bytes, data + 4, 4);
    std::memcpy(&result.value, result.raw_bytes, 4);
    result.status = (can_id >> 16) & 0xFF;
    result.success = (result.status == 0);
    result.timestamp = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lk(feedback_mutex_);
    param_results_[motor_id] = result;
}

void RobstrideElmueDriver::parseFaultFrame(const uint8_t data[8], uint8_t motor_id) {
    uint32_t fault = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    fault_details_[motor_id] = fault;
}

void RobstrideElmueDriver::parseVersionResponse(const uint8_t data[8], uint8_t motor_id) {
    FirmwareVersion fw{};
    fw.motor_id = motor_id;
    std::memcpy(fw.version_bytes, data + 2, 5);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u.%u",
                  fw.version_bytes[0], fw.version_bytes[1], fw.version_bytes[2],
                  fw.version_bytes[3], fw.version_bytes[4]);
    fw.version_str = buf;
    fw.timestamp = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lk(feedback_mutex_);
    firmware_versions_[motor_id] = fw;
}

}  // namespace elmue_can
