// zeroerr_canopen.cpp —— ZeroErr / 通用 CiA-402 CANopen 主站在 ElmueCan 上的实现

#include "elmue_can/zeroerr_canopen.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <thread>

namespace elmue_can {

struct ZeroErrCanopenDriver::FeedbackSnapshotStore {
    struct Slot {
        std::atomic<uint32_t> sequence{0};
        std::atomic<double> position{0.0};
        std::atomic<double> velocity{0.0};
        std::atomic<double> torque{0.0};
        std::atomic<int64_t> update_ns{0};
        std::atomic<uint32_t> status{0};
        std::atomic<uint32_t> flags{0};
        std::atomic<uint32_t> error_nmt_mode{0};
    };
    struct ScaleSlot {
        std::atomic<uint32_t> sequence{0};
        std::atomic<double> counts_per_rev{canopen::COUNTS_PER_REV_19BIT};
        std::atomic<double> sign{1.0};
        std::atomic<double> offset_rad{0.0};
    };
    std::array<Slot, 128> slots{};
    std::array<ScaleSlot, 128> scales{};
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

using namespace std::chrono;
using canopen::Cia402Mode;
using canopen::Cia402State;

namespace {
void sleepMs(int ms) { std::this_thread::sleep_for(milliseconds(ms)); }
int32_t rdI32LE(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}
uint16_t rdU16LE(const uint8_t* p) {
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

ZeroErrCanopenDriver::ZeroErrCanopenDriver(uint8_t master_node_id)
    : master_node_id_(master_node_id) {
    auto* store = new (std::nothrow) FeedbackSnapshotStore();
    if (!store || !feedbackRegistry().add(this, store)) {
        delete store; last_error_ = "feedback snapshot registry exhausted";
    }
}

ZeroErrCanopenDriver::~ZeroErrCanopenDriver() {
    close();
    delete static_cast<FeedbackSnapshotStore*>(feedbackRegistry().remove(this));
}

// ============================ 生命周期 ============================
bool ZeroErrCanopenDriver::init(const std::string& serial, uint32_t bitrate) {
    owned_can_ = std::make_unique<ElmueCan>();
    can_ = owned_can_.get();
    shared_ = false;

    if (!can_->open(serial)) {
        last_error_ = can_->lastError();
        can_ = nullptr;
        owned_can_.reset();
        return false;
    }
    // ZeroErr 仅支持经典 CAN；采样点 0.75 与 Robstride 共线时保持一致，保证位时序兼容。
    if (!can_->setBitrateHz(bitrate, 0.75)) {
        last_error_ = can_->lastError();
        can_->close();
        can_ = nullptr;
        owned_can_.reset();
        return false;
    }

    // 独占模式：内部 RX 回调只挑出 CANopen 标准帧转 handleRxFrame。
    can_->setRxCallback([this](uint8_t msg_type, const uint8_t* payload, size_t len) {
        if (msg_type != MSG_RxFrame) return;
        constexpr size_t kCanIdEnd = offsetof(kRxFrameElmue, timestamp);  // = 7（无时间戳布局）
        if (len < kCanIdEnd) return;
        const auto* rx = reinterpret_cast<const kRxFrameElmue*>(payload);
        if (rx->can_id & CAN_ID_29Bit) return;  // CANopen 只用标准帧（29 位留给 Robstride）
        const uint32_t cob = rx->can_id & CAN_MASK_11;
        const uint8_t dlc = static_cast<uint8_t>(len - kCanIdEnd);
        handleRxFrame(cob, payload + kCanIdEnd, dlc);
    });

    if (!can_->start(0, true)) {
        last_error_ = can_->lastError();
        can_->close();
        can_ = nullptr;
        owned_can_.reset();
        return false;
    }
    return true;
}

void ZeroErrCanopenDriver::attach(ElmueCan* shared_can) {
    owned_can_.reset();
    can_ = shared_can;
    shared_ = true;
}

void ZeroErrCanopenDriver::close() {
    if (owned_can_) {
        owned_can_->close();
        owned_can_.reset();
    }
    can_ = nullptr;
    shared_ = false;
}

bool ZeroErrCanopenDriver::isConnected() const {
    return can_ != nullptr && can_->started();
}

// ============================ 底层发送 ============================
bool ZeroErrCanopenDriver::sendCanopenFrame(uint32_t cob_id, const uint8_t* data, uint8_t dlc) {
    if (!can_) {
        last_error_ = "未连接 CAN 适配器";
        return false;
    }
    ElmueCanFrame f{};
    f.can_id = cob_id & CAN_MASK_11;
    f.extended = false;  // CANopen 标准帧
    f.fd = false;        // 经典 CAN
    f.brs = false;
    f.data_len = dlc;
    if (data && dlc > 0) {
        std::memcpy(f.data, data, dlc > 8 ? 8 : dlc);
    }
    return can_->sendFrame(f, /*marker=*/0);
}

// ============================ RX 分发 ============================
void ZeroErrCanopenDriver::handleRxFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    const uint32_t func = canopen::cobFunction(can_id);
    const uint8_t node = canopen::cobNodeId(can_id);

    switch (func) {
        case canopen::COBID_SDO_TX_BASE:  // 0x580 SDO 响应
            if (dlc >= 8) onSdoResponse(node, data);
            break;
        case canopen::COBID_TPDO1_BASE:   // 0x180
        case canopen::COBID_TPDO2_BASE:   // 0x280
        case canopen::COBID_TPDO3_BASE:   // 0x380
        case canopen::COBID_TPDO4_BASE:   // 0x480
            onTpdo(func, node, data, dlc);
            break;
        case canopen::COBID_HEARTBEAT_BASE:  // 0x700 心跳 / BootUp
            onHeartbeat(node, data, dlc);
            break;
        case canopen::COBID_EMCY_BASE:    // 0x080：node==0 为 SYNC（本机发，不处理），node!=0 为 EMCY
            if (node != 0) onEmcy(node, data, dlc);
            break;
        default:
            break;
    }
}

void ZeroErrCanopenDriver::onSdoResponse(uint8_t node_id, const uint8_t data[8]) {
    {
        std::lock_guard<std::mutex> lk(sdo_mutex_);
        SdoSlot& slot = sdo_slots_[node_id];
        slot.valid = true;
        slot.cmd = data[0];
        slot.index = canopen::sdoIndex(data);
        slot.sub = canopen::sdoSubindex(data);
        slot.data = canopen::sdoData32(data);
    }
    sdo_cv_.notify_all();
}

void ZeroErrCanopenDriver::onTpdo(uint32_t func_base, uint8_t node_id,
                                  const uint8_t* data, uint8_t dlc) {
    const canopen::ScaleConfig scale = scaleFor(node_id);
    CanopenFeedback snapshot;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        CanopenFeedback& fb = feedbacks_[node_id];
        fb.node_id = node_id;
        fb.online = true;
        fb.last_update = steady_clock::now();
        if (dlc >= 2) {
            fb.statusword = rdU16LE(data);
            fb.state = canopen::parseState(fb.statusword);
            fb.target_reached = (fb.statusword & canopen::SW_BIT_TARGET_REACHED) != 0;
        }
        if (func_base == canopen::COBID_TPDO2_BASE && dlc >= 6) {
            fb.position = scale.countsToRad(rdI32LE(data + 2));
            fb.position_valid = true;
        } else if (func_base == canopen::COBID_TPDO3_BASE && dlc >= 6) {
            fb.velocity = scale.countsToRads(rdI32LE(data + 2));
        }
        snapshot = fb;
    }
    publishFeedbackSnapshot(snapshot);
}

void ZeroErrCanopenDriver::onHeartbeat(uint8_t node_id, const uint8_t* data, uint8_t dlc) {
    if (dlc < 1) return;
    CanopenFeedback snapshot;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        CanopenFeedback& fb = feedbacks_[node_id];
        fb.node_id = node_id;
        fb.online = true;
        fb.nmt_state = data[0] & 0x7F;
        fb.last_update = steady_clock::now();
        snapshot = fb;
    }
    publishFeedbackSnapshot(snapshot);
}

void ZeroErrCanopenDriver::onEmcy(uint8_t node_id, const uint8_t* data, uint8_t dlc) {
    if (dlc < 2) return;
    CanopenFeedback snapshot;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        CanopenFeedback& fb = feedbacks_[node_id];
        fb.node_id = node_id;
        fb.online = true;
        fb.error_code = rdU16LE(data);
        fb.last_update = steady_clock::now();
        snapshot = fb;
    }
    publishFeedbackSnapshot(snapshot);
}

// ============================ 节点登记 ============================
void ZeroErrCanopenDriver::addNode(uint8_t node_id, const canopen::ScaleConfig& scale) {
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        scales_[node_id] = scale;
        CanopenFeedback& fb = feedbacks_[node_id];
        fb.node_id = node_id;
    }
    auto* snapshots = static_cast<FeedbackSnapshotStore*>(feedbackRegistry().get(this));
    if (snapshots && node_id < snapshots->scales.size()) {
        auto& slot = snapshots->scales[node_id];
        slot.sequence.fetch_add(1, std::memory_order_acq_rel);
        slot.counts_per_rev.store(scale.counts_per_rev, std::memory_order_relaxed);
        slot.sign.store(scale.sign, std::memory_order_relaxed);
        slot.offset_rad.store(scale.offset_rad, std::memory_order_relaxed);
        slot.sequence.fetch_add(1, std::memory_order_release);
    }
}

void ZeroErrCanopenDriver::setScale(uint8_t node_id, const canopen::ScaleConfig& scale) {
    addNode(node_id, scale);
}

canopen::ScaleConfig ZeroErrCanopenDriver::getScale(uint8_t node_id) const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    auto it = scales_.find(node_id);
    return it != scales_.end() ? it->second : canopen::ScaleConfig{};
}

canopen::ScaleConfig ZeroErrCanopenDriver::scaleFor(uint8_t node_id) {
    canopen::ScaleConfig scale;
    auto* snapshots = static_cast<FeedbackSnapshotStore*>(feedbackRegistry().get(this));
    if (!snapshots || node_id >= snapshots->scales.size()) return scale;
    const auto& slot = snapshots->scales[node_id];
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t before = slot.sequence.load(std::memory_order_acquire);
        if (before & 1u) continue;
        scale.counts_per_rev = slot.counts_per_rev.load(std::memory_order_relaxed);
        scale.sign = slot.sign.load(std::memory_order_relaxed);
        scale.offset_rad = slot.offset_rad.load(std::memory_order_relaxed);
        if (before == slot.sequence.load(std::memory_order_acquire)) return scale;
    }
    return canopen::ScaleConfig{};
}

// ============================ NMT ============================
bool ZeroErrCanopenDriver::nmtCommand(uint8_t command, uint8_t node_id) {
    const uint8_t data[2] = {command, node_id};
    return sendCanopenFrame(canopen::COBID_NMT, data, 2);
}
bool ZeroErrCanopenDriver::nmtStart(uint8_t node_id) { return nmtCommand(canopen::NMT_START_NODE, node_id); }
bool ZeroErrCanopenDriver::nmtStop(uint8_t node_id) { return nmtCommand(canopen::NMT_STOP_NODE, node_id); }
bool ZeroErrCanopenDriver::nmtEnterPreOp(uint8_t node_id) { return nmtCommand(canopen::NMT_ENTER_PREOP, node_id); }
bool ZeroErrCanopenDriver::nmtResetNode(uint8_t node_id) { return nmtCommand(canopen::NMT_RESET_NODE, node_id); }
bool ZeroErrCanopenDriver::nmtResetComm(uint8_t node_id) { return nmtCommand(canopen::NMT_RESET_COMM, node_id); }

// ============================ SDO ============================
SdoResult ZeroErrCanopenDriver::sdoTransfer(uint8_t node_id, const uint8_t req[8], double timeout_s) {
    SdoResult res;
    if (!can_) {
        last_error_ = "未连接 CAN 适配器";
        return res;
    }
    {
        std::lock_guard<std::mutex> lk(sdo_mutex_);
        sdo_slots_[node_id].valid = false;
    }
    if (!sendCanopenFrame(canopen::COBID_SDO_RX_BASE + node_id, req, 8)) {
        last_error_ = "SDO 请求发送失败";
        return res;
    }

    std::unique_lock<std::mutex> lk(sdo_mutex_);
    const auto dur = duration<double>(timeout_s);
    if (!sdo_cv_.wait_for(lk, dur, [&] { return sdo_slots_[node_id].valid; })) {
        last_error_ = "SDO 超时 (node " + std::to_string(node_id) + ", index 0x" +
                      std::to_string(canopen::sdoIndex(req)) + ")";
        return res;
    }
    const SdoSlot slot = sdo_slots_[node_id];
    lk.unlock();

    if (slot.cmd == canopen::SDO_ABORT) {
        res.aborted = true;
        res.abort_code = slot.data;
        last_error_ = "SDO 被中止 (abort 0x" + std::to_string(slot.data) + ")";
        return res;
    }
    if (req[0] == canopen::SDO_UL_REQ) {  // 读
        const uint8_t nb = canopen::sdoUploadRespBytes(slot.cmd);
        const uint32_t mask = (nb >= 4) ? 0xFFFFFFFFu : ((1u << (8 * nb)) - 1u);
        res.value = slot.data & mask;
        res.success = true;
    } else {  // 写：期望下载成功响应 0x60
        res.success = (slot.cmd == canopen::SDO_DL_RESP);
    }
    return res;
}

SdoResult ZeroErrCanopenDriver::sdoWrite(uint8_t node_id, uint16_t index, uint8_t sub,
                                         uint32_t value, uint8_t nbytes, double timeout_s) {
    uint8_t req[8];
    canopen::packSdoDownload(index, sub, value, nbytes, req);
    return sdoTransfer(node_id, req, timeout_s);
}

SdoResult ZeroErrCanopenDriver::sdoRead(uint8_t node_id, uint16_t index, uint8_t sub, double timeout_s) {
    uint8_t req[8];
    canopen::packSdoUpload(index, sub, req);
    return sdoTransfer(node_id, req, timeout_s);
}

// ============================ CiA-402 状态机 ============================
std::optional<uint16_t> ZeroErrCanopenDriver::readStatusword(uint8_t node_id, double timeout_s) {
    SdoResult r = sdoRead(node_id, canopen::OD_STATUSWORD, 0, timeout_s);
    if (!r.success) return std::nullopt;
    return static_cast<uint16_t>(r.value & 0xFFFF);
}

Cia402State ZeroErrCanopenDriver::readState(uint8_t node_id, double timeout_s) {
    auto sw = readStatusword(node_id, timeout_s);
    return sw ? canopen::parseState(*sw) : Cia402State::Unknown;
}

bool ZeroErrCanopenDriver::setControlword(uint8_t node_id, uint16_t controlword) {
    return sdoWrite(node_id, canopen::OD_CONTROLWORD, 0, controlword, 2).success;
}

bool ZeroErrCanopenDriver::faultReset(uint8_t node_id, double timeout_s) {
    // bit7 上升沿：先写 0x80，再回落 0x00
    if (!setControlword(node_id, canopen::CW_FAULT_RESET)) return false;
    sleepMs(50);
    setControlword(node_id, 0x0000);
    const auto deadline = steady_clock::now() + milliseconds(static_cast<int>(timeout_s * 1000));
    while (steady_clock::now() < deadline) {
        sleepMs(20);
        Cia402State st = readState(node_id, 0.2);
        if (st != Cia402State::Fault && st != Cia402State::FaultReactionActive &&
            st != Cia402State::Unknown) {
            return true;
        }
    }
    return false;
}

bool ZeroErrCanopenDriver::enable(uint8_t node_id, double timeout_s) {
    const auto deadline = steady_clock::now() + milliseconds(static_cast<int>(timeout_s * 1000));
    while (steady_clock::now() < deadline) {
        auto sw = readStatusword(node_id, 0.2);
        if (!sw) {
            sleepMs(20);
            continue;
        }
        switch (canopen::parseState(*sw)) {
            case Cia402State::Fault:
            case Cia402State::FaultReactionActive:
                faultReset(node_id, 0.5);
                break;
            case Cia402State::SwitchOnDisabled:
                setControlword(node_id, canopen::CW_SHUTDOWN);       // -> Ready to switch on
                break;
            case Cia402State::ReadyToSwitchOn:
                setControlword(node_id, canopen::CW_SWITCH_ON);      // -> Switched on
                break;
            case Cia402State::SwitchedOn:
                setControlword(node_id, canopen::CW_ENABLE_OPERATION);  // -> Operation enabled
                break;
            case Cia402State::QuickStopActive:
                setControlword(node_id, canopen::CW_ENABLE_OPERATION);
                break;
            case Cia402State::OperationEnabled:
                return true;
            default:
                setControlword(node_id, canopen::CW_SHUTDOWN);
                break;
        }
        sleepMs(10);
    }
    last_error_ = "使能超时 (node " + std::to_string(node_id) + ")";
    return false;
}

bool ZeroErrCanopenDriver::disable(uint8_t node_id) {
    return setControlword(node_id, canopen::CW_DISABLE_VOLTAGE);  // -> Switch on disabled
}

bool ZeroErrCanopenDriver::quickStop(uint8_t node_id) {
    return setControlword(node_id, canopen::CW_QUICK_STOP);
}

// ============================ 运行模式 ============================
bool ZeroErrCanopenDriver::setMode(uint8_t node_id, Cia402Mode mode, double timeout_s) {
    return sdoWrite(node_id, canopen::OD_MODES_OF_OP, 0,
                    static_cast<uint32_t>(static_cast<uint8_t>(static_cast<int8_t>(mode))), 1, timeout_s)
        .success;
}

std::optional<Cia402Mode> ZeroErrCanopenDriver::readMode(uint8_t node_id, double timeout_s) {
    SdoResult r = sdoRead(node_id, canopen::OD_MODES_DISPLAY, 0, timeout_s);
    if (!r.success) return std::nullopt;
    return static_cast<Cia402Mode>(static_cast<int8_t>(r.value & 0xFF));
}

// ============================ 运动指令 ============================
bool ZeroErrCanopenDriver::setProfileParams(uint8_t node_id, double profile_velocity_rads,
                                            double profile_acc_rads2, double profile_dec_rads2) {
    const canopen::ScaleConfig scale = scaleFor(node_id);
    bool ok = true;
    if (profile_velocity_rads > 0.0) {
        const uint32_t v = static_cast<uint32_t>(std::abs(scale.radsToCounts(profile_velocity_rads)));
        ok &= sdoWrite(node_id, canopen::OD_PROFILE_VELOCITY, 0, v, 4).success;
    }
    if (profile_acc_rads2 > 0.0) {
        const uint32_t a = static_cast<uint32_t>(std::abs(scale.radsToCounts(profile_acc_rads2)));
        ok &= sdoWrite(node_id, canopen::OD_PROFILE_ACC, 0, a, 4).success;
    }
    if (profile_dec_rads2 > 0.0) {
        const uint32_t d = static_cast<uint32_t>(std::abs(scale.radsToCounts(profile_dec_rads2)));
        ok &= sdoWrite(node_id, canopen::OD_PROFILE_DEC, 0, d, 4).success;
    }
    return ok;
}

bool ZeroErrCanopenDriver::moveToPositionPP(uint8_t node_id, double position_rad,
                                            double profile_velocity_rads,
                                            bool immediate, bool relative) {
    const canopen::ScaleConfig scale = scaleFor(node_id);
    if (profile_velocity_rads > 0.0) {
        if (!setProfileParams(node_id, profile_velocity_rads)) return false;
    }
    const int32_t counts = scale.radToCounts(position_rad);
    if (!sdoWrite(node_id, canopen::OD_TARGET_POSITION, 0, static_cast<uint32_t>(counts), 4).success) {
        return false;
    }
    uint16_t base = canopen::CW_ENABLE_OPERATION;
    if (immediate) base |= canopen::CW_BIT_CHANGE_IMMEDIATELY;
    if (relative) base |= canopen::CW_BIT_ABS_REL;

    // CiA-402 PP setpoint 握手 + 诊断：bit4(new-setpoint) 0->1 上升沿锁存新目标，
    // 从站以 bit12(setpoint-ack) 应答。拉高 bit4 前需确认 bit12 已回 0（上次锁存已释放）。
    uint16_t sw_before = 0;
    if (auto s = readStatusword(node_id, 0.1)) sw_before = *s;

    if (!setControlword(node_id, base)) return false;  // 确保 bit4=0
    bool ack_cleared = false;
    {   // 等 setpoint-ack(bit12) 清零（上一次握手释放）
        const auto dl = steady_clock::now() + milliseconds(300);
        while (steady_clock::now() < dl) {
            auto sw = readStatusword(node_id, 0.1);
            if (sw && !(*sw & canopen::SW_BIT_SETPOINT_ACK)) { ack_cleared = true; break; }
            sleepMs(5);
        }
    }
    if (!ack_cleared) return false;  // No trigger without observed ACK release.
    if (!setControlword(node_id, base | canopen::CW_BIT_NEW_SETPOINT))
        return false;  // bit4=1 触发
    bool ack_set = false;
    uint16_t sw_trig = 0;
    {   // 等 setpoint-ack(bit12) 置位（从站已接收本次目标）
        const auto dl = steady_clock::now() + milliseconds(300);
        while (steady_clock::now() < dl) {
            auto sw = readStatusword(node_id, 0.1);
            if (sw) { sw_trig = *sw; if (*sw & canopen::SW_BIT_SETPOINT_ACK) { ack_set = true; break; } }
            sleepMs(5);
        }
    }
    const bool reset_ok = setControlword(node_id, base);  // bit4=0 复位，便于下次触发
    std::fprintf(stderr,
                 "    [PP-diag] target=%d counts  base=0x%04X  sw_before=0x%04X  "
                 "ack_cleared=%d  ack_set=%d  sw_trig=0x%04X\n",
                 counts, base, sw_before, ack_cleared ? 1 : 0, ack_set ? 1 : 0, sw_trig);
    return ack_cleared && ack_set && reset_ok;
}

bool ZeroErrCanopenDriver::haltPositionPP(uint8_t node_id) {
    return setControlword(
        node_id,
        static_cast<uint16_t>(canopen::CW_ENABLE_OPERATION |
                              canopen::CW_BIT_HALT));
}

bool ZeroErrCanopenDriver::setTargetPositionCSP(uint8_t node_id, double position_rad) {
    ElmueCanFrame frame{};
    return encodeTargetPositionCSP(node_id, position_rad, frame) && can_ && can_->sendFrame(frame, 0);
}

bool ZeroErrCanopenDriver::encodeTargetPositionCSP(uint8_t node_id, double position_rad,
                                                   ElmueCanFrame& frame) {
    // 经 RPDO2(0x300+id) 下发 [Controlword(16) | TargetPosition(32)]（EDS 默认映射）。
    // 需先 configureDefaultPdos() 启用 RPDO2 且 nmtStart()，并周期性 sendSync()。
    const canopen::ScaleConfig scale = scaleFor(node_id);
    const int32_t counts = scale.radToCounts(position_rad);
    const uint16_t cw = canopen::CW_ENABLE_OPERATION;
    uint8_t d[6];
    d[0] = static_cast<uint8_t>(cw & 0xFF);
    d[1] = static_cast<uint8_t>((cw >> 8) & 0xFF);
    const uint32_t u = static_cast<uint32_t>(counts);
    d[2] = static_cast<uint8_t>(u & 0xFF);
    d[3] = static_cast<uint8_t>((u >> 8) & 0xFF);
    d[4] = static_cast<uint8_t>((u >> 16) & 0xFF);
    d[5] = static_cast<uint8_t>((u >> 24) & 0xFF);
    frame = ElmueCanFrame{};
    frame.can_id = canopen::COBID_RPDO2_BASE + node_id;
    frame.extended = false;
    frame.fd = false;
    frame.brs = false;
    frame.data_len = 6;
    std::memcpy(frame.data, d, 6);
    return true;
}

ElmueCanFrame ZeroErrCanopenDriver::encodeSyncFrame() {
    ElmueCanFrame frame{};
    frame.can_id = canopen::COBID_SYNC;
    frame.extended = false;
    frame.fd = false;
    frame.brs = false;
    frame.data_len = 0;
    return frame;
}

bool ZeroErrCanopenDriver::setTargetVelocity(uint8_t node_id, double velocity_rads) {
    const canopen::ScaleConfig scale = scaleFor(node_id);
    const int32_t counts = scale.radsToCounts(velocity_rads);
    return sdoWrite(node_id, canopen::OD_TARGET_VELOCITY, 0, static_cast<uint32_t>(counts), 4).success;
}

std::optional<double> ZeroErrCanopenDriver::readPosition(uint8_t node_id, double timeout_s) {
    SdoResult r = sdoRead(node_id, canopen::OD_POSITION_ACTUAL, 0, timeout_s);
    if (!r.success) return std::nullopt;
    const canopen::ScaleConfig scale = scaleFor(node_id);
    return scale.countsToRad(static_cast<int32_t>(r.value));
}

// ============================ PDO ============================
bool ZeroErrCanopenDriver::sendSync() {
    return sendCanopenFrame(canopen::COBID_SYNC, nullptr, 0);
}

bool ZeroErrCanopenDriver::configureDefaultPdos(uint8_t node_id, uint8_t sync_every, double timeout_s) {
    // 修改 PDO 映射必须在 Pre-Operational 下进行，并严格遵循：禁用 PDO ->
    // 映射数清零 -> 写映射项 -> 恢复映射数 -> 配置通信参数 -> 启用 PDO。
    // 不能依赖厂商默认映射；AIMotor 默认 TPDO2/RPDO2 是速度/转矩对象，
    // 若直接按 6041+6064 / 6040+607A 解析会得到错误位置并可能造成 CSP 突跳。
    nmtEnterPreOp(node_id);
    sleepMs(20);
    bool ok = true;

    const auto write = [&](uint16_t index, uint8_t sub, uint32_t value, uint8_t bytes) {
        const bool success = sdoWrite(node_id, index, sub, value, bytes, timeout_s).success;
        ok &= success;
    };
    const uint32_t tpdo2_cob = canopen::COBID_TPDO2_BASE + node_id;
    const uint32_t tpdo3_cob = canopen::COBID_TPDO3_BASE + node_id;
    const uint32_t rpdo2_cob = canopen::COBID_RPDO2_BASE + node_id;

    // TPDO2: Statusword(6041:00, 16 bit) + Position actual(6064:00, 32 bit).
    write(canopen::OD_TPDO2_COMM, canopen::PDO_COMM_SUB_COBID,
          canopen::PDO_COBID_INVALID_BIT | tpdo2_cob, 4);
    write(canopen::OD_TPDO2_MAP, 0, 0, 1);
    write(canopen::OD_TPDO2_MAP, 1, 0x60410010u, 4);
    write(canopen::OD_TPDO2_MAP, 2, 0x60640020u, 4);
    write(canopen::OD_TPDO2_MAP, 0, 2, 1);
    write(canopen::OD_TPDO2_COMM, canopen::PDO_COMM_SUB_TRANSMISSION, sync_every, 1);
    write(canopen::OD_TPDO2_COMM, canopen::PDO_COMM_SUB_COBID, tpdo2_cob, 4);

    // TPDO3: Statusword(6041:00, 16 bit) + Velocity actual(606C:00, 32 bit).
    write(canopen::OD_TPDO3_COMM, canopen::PDO_COMM_SUB_COBID,
          canopen::PDO_COBID_INVALID_BIT | tpdo3_cob, 4);
    write(canopen::OD_TPDO3_MAP, 0, 0, 1);
    write(canopen::OD_TPDO3_MAP, 1, 0x60410010u, 4);
    write(canopen::OD_TPDO3_MAP, 2, 0x606C0020u, 4);
    write(canopen::OD_TPDO3_MAP, 0, 2, 1);
    write(canopen::OD_TPDO3_COMM, canopen::PDO_COMM_SUB_TRANSMISSION, sync_every, 1);
    write(canopen::OD_TPDO3_COMM, canopen::PDO_COMM_SUB_COBID, tpdo3_cob, 4);

    // RPDO2: Controlword(6040:00, 16 bit) + Target position(607A:00, 32 bit).
    write(canopen::OD_RPDO2_COMM, canopen::PDO_COMM_SUB_COBID,
          canopen::PDO_COBID_INVALID_BIT | rpdo2_cob, 4);
    write(canopen::OD_RPDO2_MAP, 0, 0, 1);
    write(canopen::OD_RPDO2_MAP, 1, 0x60400010u, 4);
    write(canopen::OD_RPDO2_MAP, 2, 0x607A0020u, 4);
    write(canopen::OD_RPDO2_MAP, 0, 2, 1);
    write(canopen::OD_RPDO2_COMM, canopen::PDO_COMM_SUB_TRANSMISSION, sync_every, 1);
    write(canopen::OD_RPDO2_COMM, canopen::PDO_COMM_SUB_COBID, rpdo2_cob, 4);

    nmtStart(node_id);
    return ok;
}

// ============================ 反馈 ============================
CanopenFeedback ZeroErrCanopenDriver::getFeedback(uint8_t node_id) {
    CanopenFeedback fb{};
    auto* snapshots = static_cast<FeedbackSnapshotStore*>(feedbackRegistry().get(this));
    if (!snapshots || node_id >= snapshots->slots.size()) return fb;
    const auto& slot = snapshots->slots[node_id];
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t before = slot.sequence.load(std::memory_order_acquire);
        if (before & 1u) continue;
        const uint32_t flags = slot.flags.load(std::memory_order_relaxed);
        if ((flags & 0x1u) == 0) return fb;
        fb.node_id = node_id;
        fb.position = slot.position.load(std::memory_order_relaxed);
        fb.velocity = slot.velocity.load(std::memory_order_relaxed);
        fb.torque = slot.torque.load(std::memory_order_relaxed);
        fb.statusword = static_cast<uint16_t>(slot.status.load(std::memory_order_relaxed));
        fb.state = canopen::parseState(fb.statusword);
        const uint32_t packed = slot.error_nmt_mode.load(std::memory_order_relaxed);
        fb.error_code = static_cast<uint16_t>(packed & 0xffffu);
        fb.nmt_state = static_cast<uint8_t>((packed >> 16) & 0xffu);
        fb.mode_display = static_cast<int8_t>((packed >> 24) & 0xffu);
        fb.online = true;
        fb.position_valid = (flags & 0x2u) != 0;
        fb.target_reached = (flags & 0x4u) != 0;
        fb.last_update = steady_clock::time_point(
            nanoseconds(slot.update_ns.load(std::memory_order_relaxed)));
        const uint32_t after = slot.sequence.load(std::memory_order_acquire);
        if (before == after) return fb;
    }
    return CanopenFeedback{};
}

void ZeroErrCanopenDriver::publishFeedbackSnapshot(const CanopenFeedback& fb) {
    auto* snapshots = static_cast<FeedbackSnapshotStore*>(feedbackRegistry().get(this));
    if (!snapshots || fb.node_id >= snapshots->slots.size()) return;
    auto& slot = snapshots->slots[fb.node_id];
    slot.sequence.fetch_add(1, std::memory_order_acq_rel);
    slot.position.store(fb.position, std::memory_order_relaxed);
    slot.velocity.store(fb.velocity, std::memory_order_relaxed);
    slot.torque.store(fb.torque, std::memory_order_relaxed);
    slot.status.store(fb.statusword, std::memory_order_relaxed);
    slot.update_ns.store(duration_cast<nanoseconds>(fb.last_update.time_since_epoch()).count(),
                         std::memory_order_relaxed);
    slot.error_nmt_mode.store(static_cast<uint32_t>(fb.error_code) |
        (static_cast<uint32_t>(fb.nmt_state) << 16) |
        (static_cast<uint32_t>(static_cast<uint8_t>(fb.mode_display)) << 24),
        std::memory_order_relaxed);
    const uint32_t flags = (fb.online ? 0x1u : 0u) |
        (fb.position_valid ? 0x2u : 0u) | (fb.target_reached ? 0x4u : 0u);
    slot.flags.store(flags, std::memory_order_release);
    slot.sequence.fetch_add(1, std::memory_order_release);
}

std::map<uint8_t, CanopenFeedback> ZeroErrCanopenDriver::getAllFeedbacks() {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return feedbacks_;
}

}  // namespace elmue_can
