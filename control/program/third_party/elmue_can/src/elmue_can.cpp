#include <unistd.h>
// elmue_can.cpp —— ElmueCan libusb 用户态驱动实现
//
// 抽取自 rs00_comm_test.cpp（第 355-895 行）。

#include "elmue_can/elmue_can.h"

#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>

#include <pthread.h>
#include <sched.h>

namespace elmue_can {

namespace {

constexpr int kUsbTimeoutMs = 100;
constexpr int kRxBufSize = 2048;

// -----------------------------------------------------------------------------
// libusb_context 生命周期策略（针对 PREEMPT_RT + libusb 1.0.25 的多实例竞态）
//
// 每个 ElmueCan 实例仍持有**独立**的 libusb_context（保留原始设计）。区别：
//   - 单 ctx 共享方案：多个 rxLoop + sendFramesBatch 并发访问同 ctx 的 hotplug
//     / event 链表锁，触发 `usbi_mutex_lock: Assertion pthread_mutex_lock(mutex)
//     == 0 failed`（libusb 1.0.25 的已知内部竞态）。
//   - 独立 ctx + libusb_exit：exit 与飞行 URB callback 的 race，PREEMPT_RT
//     调度让窗口更宽，多 CANable 会 SEGV。
//
// 权衡：保留独立 ctx（避免跨实例 mutex race），但**永不 libusb_exit**，让每个
// ctx 常驻至进程退出（OS 自动回收 usbfs）。
//   - 泄漏 ~几 KB × #open 次数；对 scan/mirror 类进程完全可接受。
//   - 若上层需多次 open/close 同一 ElmueCan 实例，close 只释放 usb handle，
//     不销毁 ctx；下次 open 重新 libusb_init 一个新 ctx。
// -----------------------------------------------------------------------------
libusb_context *initIsolatedCtx(std::string &err) {
    libusb_context *ctx = nullptr;
    int rc = libusb_init(&ctx);
    if (rc != 0) {
        err = std::string("libusb_init: ") + libusb_error_name(rc);
        return nullptr;
    }
    return ctx;
}

uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi) {
    return std::max(lo, std::min(v, hi));
}

void apply_thread_rt(int prio, int cpu) {
    if (prio > 0) {
        struct sched_param sp{};
        sp.sched_priority = prio;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }
    if (cpu >= 0) {
#ifdef __linux__
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
#else
        (void)cpu;
#endif
    }
}

// 批量异步发送的共享完成上下文（多个 transfer 共用一个）
struct BatchTxCtx {
    std::atomic<int> done{0};
    std::atomic<int> errors{0};
};

void LIBUSB_CALL batchTxCallback(libusb_transfer *xfer) {
    auto *c = static_cast<BatchTxCtx *>(xfer->user_data);
    if (xfer->status != LIBUSB_TRANSFER_COMPLETED || xfer->actual_length != xfer->length) {
        c->errors.fetch_add(1, std::memory_order_relaxed);
    }
    c->done.fetch_add(1, std::memory_order_acq_rel);
}

constexpr int kMaxBatchFrames = 32;

template <std::size_t Capacity>
class PointerRegistry {
public:
    PointerRegistry() {
        for (auto& key : keys_) key.store(nullptr, std::memory_order_relaxed);
        for (auto& value : values_) value.store(nullptr, std::memory_order_relaxed);
    }
    void* get(const void* key) const {
        for (std::size_t i = 0; i < Capacity; ++i)
            if (keys_[i].load(std::memory_order_acquire) == key)
                return values_[i].load(std::memory_order_acquire);
        return nullptr;
    }
    bool add(const void* key, void* value) {
        for (std::size_t i = 0; i < Capacity; ++i) {
            const void* expected = nullptr;
            if (keys_[i].compare_exchange_strong(expected, key, std::memory_order_acq_rel)) {
                values_[i].store(value, std::memory_order_release);
                return true;
            }
        }
        return false;
    }
    void* remove(const void* key) {
        for (std::size_t i = 0; i < Capacity; ++i) {
            if (keys_[i].load(std::memory_order_acquire) == key) {
                void* value = values_[i].exchange(nullptr, std::memory_order_acq_rel);
                keys_[i].store(nullptr, std::memory_order_release);
                return value;
            }
        }
        return nullptr;
    }
private:
    std::array<std::atomic<const void*>, Capacity> keys_{};
    std::array<std::atomic<void*>, Capacity> values_{};
};

PointerRegistry<32>& batchRegistry() {
    static PointerRegistry<32> registry;
    return registry;
}

}  // namespace

struct ElmueCan::BatchRuntime {
    static constexpr int kBufCap = static_cast<int>(sizeof(kTxFrameElmue)) + 64;
    std::array<std::array<uint8_t, kBufCap>, kMaxBatchFrames> buffers{};
    std::array<int, kMaxBatchFrames> lengths{};
    std::array<libusb_transfer*, kMaxBatchFrames> transfers{};
    std::array<bool, kMaxBatchFrames> active{};
    BatchTxCtx completion;
    int in_flight = 0;
    bool poisoned = false;

    BatchRuntime() {
        transfers.fill(nullptr);
        for (auto& transfer : transfers) transfer = libusb_alloc_transfer(0);
    }

    ~BatchRuntime() {
        if (poisoned) return;  // Active libusb transfers are intentionally leaked on an unrecoverable drain failure.
        for (auto* transfer : transfers) {
            if (transfer) libusb_free_transfer(transfer);
        }
    }
};

ElmueCan::ElmueCan() = default;

ElmueCan::~ElmueCan() {
    close();
    auto* runtime = static_cast<BatchRuntime*>(batchRegistry().remove(this));
    if (runtime && !runtime->poisoned) delete runtime;
}

bool ElmueCan::getLastFeedback() {
    uint8_t fb = 0;
    if (!ctrlIn(ELM_ReqGetLastError, channel_, &fb, sizeof(fb))) {
        return false;
    }
    if (fb != FBK_Success) {
        last_error_ = "firmware feedback=" + std::to_string(fb);
        return false;
    }
    return true;
}

bool ElmueCan::ctrlOut(uint8_t request, uint16_t value, const void *data, uint16_t len) {
    if (!handle_) {
        last_error_ = "device not open";
        return false;
    }

    int rc = libusb_control_transfer(
        handle_,
        static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT),
        request,
        value,
        interface_,
        const_cast<unsigned char *>(static_cast<const unsigned char *>(data)),
        len,
        1000);

    if (rc < 0) {
        last_error_ = std::string("ctrlOut req=") + std::to_string(request) + " failed: " + libusb_error_name(rc);
        return false;
    }

    if (!getLastFeedback()) {
        last_error_ = std::string("ctrlOut req=") + std::to_string(request) + ": " + last_error_;
        return false;
    }
    return true;
}

bool ElmueCan::ctrlIn(uint8_t request, uint16_t value, void *data, uint16_t len) {
    if (!handle_) {
        last_error_ = "device not open";
        return false;
    }

    int rc = libusb_control_transfer(
        handle_,
        static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN),
        request,
        value,
        interface_,
        static_cast<unsigned char *>(data),
        len,
        1000);

    if (rc < 0) {
        last_error_ = std::string("ctrlIn req=") + std::to_string(request) + " failed: " + libusb_error_name(rc);
        return false;
    }
    if (static_cast<int>(len) > rc) {
        last_error_ = std::string("ctrlIn req=") + std::to_string(request) + " short read";
        return false;
    }

    if (request != ELM_ReqGetLastError) {
        if (!getLastFeedback()) {
            last_error_ = std::string("ctrlIn req=") + std::to_string(request) + ": " + last_error_;
            return false;
        }
    }
    return true;
}

bool ElmueCan::addHostFilter11(uint32_t filter, uint32_t mask) {
    kFilter f{};
    f.Operation = FIL_HostPass_11;
    f.Filter = filter;
    f.Mask = mask;
    return ctrlOut(ELM_ReqSetFilter, channel_, &f, sizeof(f));
}

bool ElmueCan::addHostFilter29(uint32_t filter, uint32_t mask) {
    kFilter f{};
    f.Operation = FIL_HostPass_29;
    f.Filter = filter;
    f.Mask = mask;
    return ctrlOut(ELM_ReqSetFilter, channel_, &f, sizeof(f));
}

void ElmueCan::setRxThreadPriority(int prio, int cpu) {
    rx_rt_prio_ = prio;
    rx_cpu_ = cpu;
}

bool ElmueCan::prepareTxFrame(const ElmueCanFrame &frame, uint8_t *out_buf, int out_cap, int *out_len) {
    if (!out_buf || !out_len || out_cap < static_cast<int>(sizeof(kTxFrameElmue))) {
        last_error_ = "prepareTxFrame invalid args";
        return false;
    }

    uint32_t can_id = frame.can_id & (frame.extended ? CAN_MASK_29 : CAN_MASK_11);
    if (frame.extended) {
        can_id |= CAN_ID_29Bit;
    }
    if (frame.rtr) {
        can_id |= CAN_ID_RTR;
    }

    uint8_t data_len = frame.data_len;
    if (frame.fd) {
        data_len = std::min<uint8_t>(data_len, 64);
    } else {
        data_len = std::min<uint8_t>(data_len, 8);
    }

    const int total = static_cast<int>(sizeof(kTxFrameElmue) + data_len);
    if (out_cap < total) {
        last_error_ = "prepareTxFrame buffer too small";
        return false;
    }

    auto *tx = reinterpret_cast<kTxFrameElmue *>(out_buf);
    tx->header.msg_type = MSG_TxFrame;
    tx->header.size = static_cast<uint8_t>(total);
    tx->can_id = can_id;
    tx->flags = 0;
    tx->marker = echo_marker_ ? echo_marker_ : 1;
    if (frame.fd) {
        tx->flags |= FRM_FDF;
    }
    if (frame.brs) {
        tx->flags |= FRM_BRS;
    }
    if (data_len > 0) {
        std::memcpy(out_buf + sizeof(kTxFrameElmue), frame.data, data_len);
    }

    echo_marker_++;
    if (echo_marker_ == 0) {
        echo_marker_ = 1;
    }
    *out_len = total;
    return true;
}

bool ElmueCan::sendPrepared(const uint8_t *buf, int len) {
    if (socket_fd_ >= 0) { last_error_ = "SocketCAN prepared USB frames unsupported; use sendFrame"; return false; }
    if (!started_ || !buf || len <= 0) {
        last_error_ = "sendPrepared invalid state";
        return false;
    }

    int transferred = 0;
    const int rc = libusb_bulk_transfer(handle_, ELMUE_EP_OUT, const_cast<uint8_t *>(buf), len, &transferred,
                                        kUsbTimeoutMs);
    if (rc != 0 || transferred != len) {
        tx_errors_++;
        last_error_ = std::string("bulk OUT failed: ") + libusb_error_name(rc);
        return false;
    }
    tx_count_++;
    return true;
}

bool ElmueCan::queryCapabilities() {
    kCapabilityClassic cap{};
    if (!ctrlIn(GS_ReqGetCapabilities, channel_, &cap, sizeof(cap))) {
        return false;
    }

    fclk_can_ = cap.fclk_can;
    nom_limits_ = cap.time;
    supports_elmue_ = (cap.feature & ELM_DevFlagProtocolElmue) != 0;
    supports_fd_ = (cap.feature & GS_DevFlagCAN_FD) != 0;
    supports_termination_ = (cap.feature & GS_DevFlagTermination) != 0;

    if (supports_fd_) {
        kCapabilityFD cap_fd{};
        if (ctrlIn(GS_ReqGetCapabilitiesFD, channel_, &cap_fd, sizeof(cap_fd))) {
            data_limits_ = cap_fd.time_data;
        }
    }
    return true;
}

bool ElmueCan::calcTiming(uint32_t target_baud, double sample_point, uint32_t &brp, uint32_t &seg1, uint32_t &seg2,
                          const kTimeMinMax &limits) {
    if (target_baud == 0 || fclk_can_ == 0) {
        last_error_ = "invalid baud or fclk";
        return false;
    }

    uint32_t best_err = UINT32_MAX;
    bool found = false;

    for (uint32_t try_brp = limits.brp_min; try_brp <= limits.brp_max; ++try_brp) {
        const uint64_t total_tq = static_cast<uint64_t>(fclk_can_) / (static_cast<uint64_t>(try_brp) * target_baud);
        if (total_tq < 8 || total_tq > 385) {
            continue;
        }

        const uint32_t tq = static_cast<uint32_t>(total_tq);
        const uint32_t try_seg1 =
            clampU32(static_cast<uint32_t>(sample_point * tq) - 1, limits.seg1_min, limits.seg1_max);
        if (try_seg1 + 2 >= tq) {
            continue;
        }
        const uint32_t try_seg2 = tq - try_seg1 - 1;
        if (try_seg2 < limits.seg2_min || try_seg2 > limits.seg2_max) {
            continue;
        }

        const uint32_t actual_baud = fclk_can_ / (try_brp * tq);
        const uint32_t err = actual_baud > target_baud ? actual_baud - target_baud : target_baud - actual_baud;
        if (err < best_err) {
            best_err = err;
            brp = try_brp;
            seg1 = try_seg1;
            seg2 = try_seg2;
            found = true;
        }
    }

    if (!found) {
        last_error_ = "failed to calculate bit timing";
        return false;
    }
    return true;
}

bool ElmueCan::setTiming(bool fd, uint32_t brp, uint32_t seg1, uint32_t seg2) {
    kBitTiming timing{};
    timing.brp = brp;
    timing.prop = 0;
    timing.seg1 = seg1;
    timing.seg2 = seg2;
    timing.sjw = std::min(seg1, seg2);

    const uint8_t req = fd ? GS_ReqSetBitTimingFD : GS_ReqSetBitTiming;
    return ctrlOut(req, channel_, &timing, sizeof(timing));
}

bool ElmueCan::setBitrateHz(uint32_t baud, double sample_point) {
    if (socket_fd_ >= 0) return checkSocketCanBitrate(baud);
    uint32_t brp = 0, seg1 = 0, seg2 = 0;
    if (!calcTiming(baud, sample_point, brp, seg1, seg2, nom_limits_)) {
        return false;
    }
    return setTiming(false, brp, seg1, seg2);
}

bool ElmueCan::setTermination(bool on) {
    // gs_usb 终端电阻请求：data = uint32 state（0=关，1=开），value=通道
    uint32_t state = on ? 1u : 0u;
    return ctrlOut(GS_ReqSetTermination, channel_, &state, sizeof(state));
}

bool ElmueCan::setBitrateFDHz(uint32_t baud, double sample_point) {
    if (!supports_fd_) {
        last_error_ = "CAN FD not supported";
        return false;
    }
    uint32_t brp = 0, seg1 = 0, seg2 = 0;
    if (!calcTiming(baud, sample_point, brp, seg1, seg2, data_limits_)) {
        return false;
    }
    return setTiming(true, brp, seg1, seg2);
}

bool ElmueCan::open(const std::string &serial) {
    close();
    if (serial.rfind("socketcan:", 0) == 0) return openSocketCan(serial.substr(10));

    ctx_ = initIsolatedCtx(last_error_);
    if (ctx_ == nullptr) {
        return false;
    }
    // 标记本实例已持有 ctx（用于 close 时释放；ctx 本身永不 libusb_exit）
    ctx_shared_ = true;

    int rc = 0;
    libusb_device **devs = nullptr;
    const ssize_t cnt = libusb_get_device_list(ctx_, &devs);
    if (cnt < 0) {
        last_error_ = std::string("libusb_get_device_list: ") + libusb_error_name(static_cast<int>(cnt));
        // ctx_ 常驻（永不 libusb_exit）；仅清空指针
        ctx_shared_ = false;
        ctx_ = nullptr;
        return false;
    }

    libusb_device_handle *picked = nullptr;
    std::string picked_serial;

    for (ssize_t i = 0; i < cnt; ++i) {
        libusb_device *dev = devs[i];
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(dev, &desc) != 0) {
            continue;
        }
        if (desc.idVendor != ELMUE_VENDOR_ID || desc.idProduct != ELMUE_PRODUCT_ID) {
            continue;
        }

        libusb_device_handle *h = nullptr;
        rc = libusb_open(dev, &h);
        if (rc != 0) {
            continue;
        }

        unsigned char buf[64]{};
        const int slen = libusb_get_string_descriptor_ascii(h, desc.iSerialNumber, buf, sizeof(buf));
        std::string dev_serial = slen > 0 ? std::string(reinterpret_cast<char *>(buf), slen) : "";

        if (!serial.empty() && dev_serial != serial) {
            libusb_close(h);
            continue;
        }

        picked = h;
        picked_serial = dev_serial;
        break;
    }

    libusb_free_device_list(devs, 1);

    if (!picked) {
        last_error_ = serial.empty() ? "no CANable device found" : "serial not found: " + serial;
        // ctx_ 常驻（永不 libusb_exit）；仅清空指针
        ctx_shared_ = false;
        ctx_ = nullptr;
        return false;
    }

    handle_ = picked;
    serial_ = picked_serial;

    if (libusb_kernel_driver_active(handle_, interface_)) {
        rc = libusb_detach_kernel_driver(handle_, interface_);
        if (rc != 0 && rc != LIBUSB_ERROR_NOT_FOUND && rc != LIBUSB_ERROR_NOT_SUPPORTED) {
            last_error_ = std::string("detach kernel driver: ") + libusb_error_name(rc);
            close();
            return false;
        }
        kernel_detached_ = (rc == 0);
    }

    rc = libusb_claim_interface(handle_, interface_);
    if (rc != 0) {
        last_error_ = std::string("claim interface: ") + libusb_error_name(rc);
        close();
        return false;
    }

    // 与官方 demo 同步: 先 Reset(), 再查询能力; 不调用 SetHostFormat
    kDeviceMode reset_mode{};
    reset_mode.mode = GS_ModeReset;
    reset_mode.flags = ELM_DevFlagProtocolElmue;
    if (!ctrlOut(GS_ReqSetDeviceMode, channel_, &reset_mode, sizeof(reset_mode))) {
        close();
        return false;
    }

    if (!queryCapabilities()) {
        close();
        return false;
    }

    if (!supports_elmue_) {
        last_error_ = "device does not support ElmueSoft protocol";
        close();
        return false;
    }

    kDeviceVersion ver{};
    ctrlIn(GS_ReqGetDeviceVersion, channel_, &ver, sizeof(ver));

    kBoardInfo board{};
    ctrlIn(ELM_ReqGetBoardInfo, channel_, &board, sizeof(board));

    return true;
}

bool ElmueCan::start(uint32_t extra_flags, bool disable_tx_echo) {
    if (socket_fd_ >= 0) {
        if (started_) return true;
        if (!checkSocketCanBitrate(1000000)) return false;
        started_ = true; rx_running_ = true;
        rx_thread_ = std::thread(&ElmueCan::socketCanRxLoop, this);
        return true;
    }
    if (!handle_) {
        last_error_ = "device not open";
        return false;
    }

    kDeviceMode mode{};
    mode.mode = GS_ModeStart;
    mode.flags = ELM_DevFlagProtocolElmue | extra_flags;
    if (disable_tx_echo) {
        mode.flags |= ELM_DevFlagDisableTxEcho;
    }

    if (!ctrlOut(GS_ReqSetDeviceMode, channel_, &mode, sizeof(mode))) {
        return false;
    }

    started_ = true;
    rx_running_ = true;
    rx_accum_len_ = 0;
    rx_thread_ = std::thread(&ElmueCan::rxLoop, this);
    return true;
}

bool ElmueCan::reset() {
    if (socket_fd_ >= 0) {
        rx_running_ = false;
        if (rx_thread_.joinable()) rx_thread_.join();
        started_ = false;
        return true;
    }
    if (!handle_) {
        return true;
    }

    if (rx_running_) {
        rx_running_ = false;
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
    }
    started_ = false;

    kDeviceMode mode{};
    mode.mode = GS_ModeReset;
    mode.flags = ELM_DevFlagProtocolElmue;
    return ctrlOut(GS_ReqSetDeviceMode, channel_, &mode, sizeof(mode));
}

void ElmueCan::close() {
    if (socket_fd_ >= 0) { reset(); ::close(socket_fd_); socket_fd_ = -1; serial_.clear(); }
    auto* batch_runtime = static_cast<BatchRuntime*>(batchRegistry().get(this));
    if (batch_runtime && batch_runtime->in_flight > 0 && ctx_) {
        auto& rt = *batch_runtime;
        for (int i = 0; i < kMaxBatchFrames; ++i)
            if (rt.active[i]) libusb_cancel_transfer(rt.transfers[i]);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
        while (rt.completion.done.load(std::memory_order_acquire) < rt.in_flight &&
               std::chrono::steady_clock::now() < deadline) {
            struct timeval tv{0, 200};
            libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
        }
        rt.poisoned = rt.completion.done.load(std::memory_order_acquire) < rt.in_flight;
        if (!rt.poisoned) { rt.in_flight = 0; rt.active.fill(false); }
    }
    if (started_ || rx_running_) {
        reset();
    }

    if (handle_) {
        libusb_release_interface(handle_, interface_);
        // 保持内核驱动 detach 状态; 如需恢复 SocketCAN 请运行 elmue_can/tools/unbind_gs_usb.sh rebind
        kernel_detached_ = false;
        libusb_close(handle_);
        handle_ = nullptr;
    }

    // ctx_ 常驻（永不 libusb_exit）；仅清空指针避免 dangling
    ctx_shared_ = false;
    ctx_ = nullptr;

    started_ = false;
    serial_.clear();
}

bool ElmueCan::sendFrame(const ElmueCanFrame &frame, uint8_t marker) {
    if (socket_fd_ >= 0) return sendSocketCan(frame);
    uint8_t buf[sizeof(kTxFrameElmue) + 64]{};
    int len = 0;
    const uint8_t saved = echo_marker_;
    if (!prepareTxFrame(frame, buf, sizeof(buf), &len)) {
        return false;
    }
    if (marker != 0) {
        auto *tx = reinterpret_cast<kTxFrameElmue *>(buf);
        tx->marker = marker;
        echo_marker_ = saved;
    }
    return sendPrepared(buf, len);
}

bool ElmueCan::sendFramesBatch(const ElmueCanFrame *frames, int count, int timeout_ms) {
    const uint32_t timeout_us = static_cast<uint32_t>(std::max(timeout_ms, 1)) * 1000u;
    return sendFramesBatchRealtime(frames, count, timeout_us).success();
}

bool ElmueCan::prepareBatchRuntime() {
    if (socket_fd_ >= 0) { last_error_ = "SocketCAN USB batch runtime unsupported"; return false; }
    auto* runtime = static_cast<BatchRuntime*>(batchRegistry().get(this));
    if (!runtime) {
        auto* created = new (std::nothrow) BatchRuntime();
        if (!created || !batchRegistry().add(this, created)) {
            delete created;
            last_error_ = "prepareBatchRuntime: registry allocation failed";
            return false;
        }
        runtime = created;
    }
    for (auto* transfer : runtime->transfers) {
        if (!transfer) {
            last_error_ = "prepareBatchRuntime: libusb_alloc_transfer failed";
            auto* removed = static_cast<BatchRuntime*>(batchRegistry().remove(this));
            delete removed;
            return false;
        }
    }
    return true;
}

BatchSendResult ElmueCan::sendFramesBatchRealtime(const ElmueCanFrame *frames, int count,
                                                   uint32_t deadline_us) {
    BatchSendResult result;
    result.requested = std::max(0, std::min(count, kMaxBatchFrames));
    const auto started_at = std::chrono::steady_clock::now();
    if (!started_ || !handle_ || !ctx_ || !frames || count <= 0 || deadline_us == 0 ||
        !prepareBatchRuntime()) {
        last_error_ = "sendFramesBatchRealtime invalid state";
        result.errors = result.requested > 0 ? result.requested : 1;
        return result;
    }

    auto* runtime = static_cast<BatchRuntime*>(batchRegistry().get(this));
    if (!runtime || runtime->poisoned) {
        last_error_ = "sendFramesBatchRealtime poisoned runtime";
        result.errors = result.requested;
        return result;
    }
    count = result.requested;
    auto& rt = *runtime;
    rt.completion.done.store(0, std::memory_order_relaxed);
    rt.completion.errors.store(0, std::memory_order_relaxed);
    rt.active.fill(false);
    const unsigned usb_timeout_ms = std::max(1u, (deadline_us + 999u) / 1000u);

    for (int i = 0; i < count; ++i) {
        int len = 0;
        if (!prepareTxFrame(frames[i], rt.buffers[i].data(), BatchRuntime::kBufCap, &len)) continue;
        rt.lengths[i] = len;
        libusb_fill_bulk_transfer(rt.transfers[i], handle_, ELMUE_EP_OUT,
                                  rt.buffers[i].data(), len, &batchTxCallback,
                                  &rt.completion, usb_timeout_ms);
        if (libusb_submit_transfer(rt.transfers[i]) == 0) {
            rt.active[i] = true;
            ++result.submitted;
        }
    }

    if (result.submitted == 0) {
        tx_errors_.fetch_add(1, std::memory_order_relaxed);
        last_error_ = "sendFramesBatchRealtime: no transfer submitted";
        result.errors = count;
        return result;
    }
    rt.in_flight = result.submitted;

    const auto deadline = started_at + std::chrono::microseconds(deadline_us);
    while (rt.completion.done.load(std::memory_order_acquire) < result.submitted &&
           std::chrono::steady_clock::now() < deadline) {
        struct timeval tv{0, 100};
        const int rc = libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
        if (rc < 0 && rc != LIBUSB_ERROR_INTERRUPTED) break;
    }

    result.completed = rt.completion.done.load(std::memory_order_acquire);
    result.timed_out = result.completed < result.submitted;
    if (result.timed_out) {
        for (int i = 0; i < count; ++i) {
            if (rt.active[i]) libusb_cancel_transfer(rt.transfers[i]);
        }
        const auto cancel_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(15);
        while (rt.completion.done.load(std::memory_order_acquire) < result.submitted &&
               std::chrono::steady_clock::now() < cancel_deadline) {
            struct timeval tv{0, 100};
            libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
        }
        result.completed = rt.completion.done.load(std::memory_order_acquire);
        rt.poisoned = result.completed < result.submitted;
    }

    if (result.completed >= result.submitted) {
        rt.in_flight = 0;
        rt.active.fill(false);
    }

    result.errors = rt.completion.errors.load(std::memory_order_acquire) +
                    (result.requested - result.submitted);
    result.duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - started_at).count();
    if (result.errors > 0 || result.timed_out) {
        tx_errors_.fetch_add(static_cast<uint64_t>(std::max(result.errors, 1)),
                             std::memory_order_relaxed);
    }
    const int succeeded = std::max(0, result.completed -
        rt.completion.errors.load(std::memory_order_acquire));
    tx_count_.fetch_add(static_cast<uint64_t>(succeeded), std::memory_order_relaxed);
    return result;
}

void ElmueCan::setRxCallback(ElmueRxCallback cb) { rx_callback_ = std::move(cb); }

void ElmueCan::processRxBuffer(const uint8_t *data, int len) {
    if (len <= 0) {
        return;
    }

    if (rx_accum_len_ + static_cast<size_t>(len) > sizeof(rx_accum_)) {
        rx_accum_len_ = 0;
        rx_errors_++;
    }

    std::memcpy(rx_accum_ + rx_accum_len_, data, static_cast<size_t>(len));
    rx_accum_len_ += static_cast<size_t>(len);

    size_t offset = 0;
    while (offset + sizeof(kHeader) <= rx_accum_len_) {
        const auto *hdr = reinterpret_cast<const kHeader *>(rx_accum_ + offset);
        if (hdr->size < sizeof(kHeader) || hdr->size > 128) {
            rx_accum_len_ = 0;
            rx_errors_++;
            break;
        }
        if (offset + hdr->size > rx_accum_len_) {
            break;
        }

        if (rx_callback_) {
            rx_callback_(hdr->msg_type, rx_accum_ + offset, hdr->size);
        }
        offset += hdr->size;
    }

    if (offset > 0) {
        const size_t remain = rx_accum_len_ - offset;
        if (remain > 0) {
            std::memmove(rx_accum_, rx_accum_ + offset, remain);
        }
        rx_accum_len_ = remain;
    }
}

void ElmueCan::rxLoop() {
    apply_thread_rt(rx_rt_prio_, rx_cpu_);
    std::vector<uint8_t> buf(kRxBufSize);

    while (rx_running_.load()) {
        int transferred = 0;
        const int rc = libusb_bulk_transfer(handle_, ELMUE_EP_IN, buf.data(), static_cast<int>(buf.size()),
                                            &transferred, kUsbTimeoutMs);

        if (!rx_running_.load()) {
            break;
        }

        if (rc == LIBUSB_ERROR_TIMEOUT) {
            continue;
        }
        if (rc != 0) {
            rx_errors_++;
            continue;
        }

        rx_usb_count_++;
        processRxBuffer(buf.data(), transferred);
    }
}

}  // namespace elmue_can
