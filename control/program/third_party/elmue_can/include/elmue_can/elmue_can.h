// elmue_can.h —— ElmueCan libusb 用户态驱动类声明
//
// 抽取自 rs00_comm_test.cpp（第 258-353 行），归属上游：
//   elmue/elmue_can.h — ElmüSoft GitHub 公开实现

#ifndef ELMUE_CAN__ELMUE_CAN_H_
#define ELMUE_CAN__ELMUE_CAN_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "elmue_can/elmue_proto.h"

struct libusb_context;
struct libusb_device_handle;

namespace elmue_can {

struct ElmueCanFrame {
    uint32_t can_id = 0;
    uint8_t data[64]{};
    uint8_t data_len = 0;
    bool extended = false;
    bool fd = false;
    bool brs = false;
    bool rtr = false;
};

struct BatchSendResult {
    int requested = 0;
    int submitted = 0;
    int completed = 0;
    int errors = 0;
    int64_t duration_us = 0;
    bool timed_out = false;

    bool success() const {
        return requested > 0 && submitted == requested && completed == submitted &&
               errors == 0 && !timed_out;
    }
};

using ElmueRxCallback = std::function<void(uint8_t msg_type, const uint8_t *payload, size_t len)>;

class ElmueCan {
public:
    ElmueCan();
    ~ElmueCan();

    ElmueCan(const ElmueCan &) = delete;
    ElmueCan &operator=(const ElmueCan &) = delete;

    bool open(const std::string &serial = "");
    void close();

    // 仲裁段采样点默认 0.75：与 Linux 内核 gs_usb 默认一致。达妙 DM4340 等 CAN FD 节点的
    // 反馈帧在 0.80 采样点下仲裁段会位采样失败（STUFF 错误，整帧丢弃），0.75 才能正确接收。
    bool setBitrateHz(uint32_t baud, double sample_point = 0.75);
    bool setBitrateFDHz(uint32_t baud, double sample_point = 0.75);
    bool start(uint32_t extra_flags = 0, bool disable_tx_echo = true);
    bool reset();

    // 开/关 CANable 内置 120Ω 终端电阻（需固件支持，见 supportsTermination()）
    bool setTermination(bool on);
    bool supportsTermination() const { return supports_termination_; }

    bool sendFrame(const ElmueCanFrame &frame, uint8_t marker = 1);
    bool sendPrepared(const uint8_t *buf, int len);

    // 批量异步发送：一次性提交 count 帧并行 USB bulk 传输（libusb_submit_transfer），
    // 重叠 USB 调度延迟，把 N 次串行往返压成 ~1 次的耗时。用于高频多电机控制循环。
    // 返回 true 表示全部帧在超时内成功完成。线程安全：仅可在单一发送线程调用。
    // timeout_ms 须小于 ros2_control 单周期（500Hz≈2ms）；默认 4ms 为 USB 并行提交的余量
    bool sendFramesBatch(const ElmueCanFrame *frames, int count, int timeout_ms = 4);

    // 500Hz 实时路径：transfer/buffer 在 prepareBatchRuntime() 中一次性预分配，
    // 每次调用仅重新编码与提交。deadline_us 是整批完成的用户态 deadline。
    bool prepareBatchRuntime();
    BatchSendResult sendFramesBatchRealtime(const ElmueCanFrame *frames, int count,
                                            uint32_t deadline_us = 1200);

    void setRxCallback(ElmueRxCallback cb);
    void setRxThreadPriority(int prio, int cpu = -1);
    bool addHostFilter11(uint32_t filter, uint32_t mask);
    bool addHostFilter29(uint32_t filter, uint32_t mask);

    bool prepareTxFrame(const ElmueCanFrame &frame, uint8_t *out_buf, int out_cap, int *out_len);

    bool started() const { return started_; }
    bool supportsFd() const { return supports_fd_; }
    const std::string &serial() const { return serial_; }
    const std::string &lastError() const { return last_error_; }
    uint32_t fclkCan() const { return fclk_can_; }

    uint64_t txCount() const { return tx_count_.load(); }
    uint64_t rxUsbCount() const { return rx_usb_count_.load(); }
    uint64_t txErrors() const { return tx_errors_.load(); }
    uint64_t rxErrors() const { return rx_errors_.load(); }

private:
    bool openSocketCan(const std::string& name);
    bool checkSocketCanBitrate(uint32_t baud);
    bool sendSocketCan(const ElmueCanFrame& frame);
    void socketCanRxLoop();
    int socket_fd_ = -1;
    int socket_index_ = 0;
    std::atomic<bool> socket_fault_{false};
    struct BatchRuntime;
    bool ctrlOut(uint8_t request, uint16_t value, const void *data, uint16_t len);
    bool ctrlIn(uint8_t request, uint16_t value, void *data, uint16_t len);
    bool getLastFeedback();
    bool queryCapabilities();
    bool setTiming(bool fd, uint32_t brp, uint32_t seg1, uint32_t seg2);
    bool calcTiming(uint32_t target_baud, double sample_point, uint32_t &brp, uint32_t &seg1, uint32_t &seg2,
                    const kTimeMinMax &limits);

    void rxLoop();
    void processRxBuffer(const uint8_t *data, int len);

    libusb_context *ctx_ = nullptr;
    // ctx_ 指向进程内共享的 libusb_context（见 elmue_can.cpp 顶部 acquireSharedCtx）；
    // 标记 true 表示本实例仍持有一份共享 ctx 的引用，close() 时需 releaseSharedCtx()。
    bool ctx_shared_ = false;
    libusb_device_handle *handle_ = nullptr;
    std::thread rx_thread_;
    std::atomic<bool> rx_running_{false};

    std::string serial_;
    std::string last_error_;
    ElmueRxCallback rx_callback_;

    uint8_t channel_ = 0;
    uint8_t interface_ = 0;
    bool kernel_detached_ = false;
    bool started_ = false;
    bool supports_fd_ = false;
    bool supports_termination_ = false;
    bool supports_elmue_ = false;
    uint32_t fclk_can_ = 0;
    kTimeMinMax nom_limits_{};
    kTimeMinMax data_limits_{};

    std::atomic<uint64_t> tx_count_{0};
    std::atomic<uint64_t> rx_usb_count_{0};
    std::atomic<uint64_t> tx_errors_{0};
    std::atomic<uint64_t> rx_errors_{0};
    std::atomic<uint64_t> rx_overflow_{0};

    int rx_rt_prio_ = 60;
    int rx_cpu_ = -1;

    uint8_t echo_marker_ = 1;
    uint8_t rx_accum_[4096]{};
    size_t rx_accum_len_ = 0;
};

}  // namespace elmue_can

#endif  // ELMUE_CAN__ELMUE_CAN_H_
