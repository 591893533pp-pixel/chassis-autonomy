#pragma once
#include <cstdint>
#include <deque>
#include <mutex>

// Receiver owns no estimator state. The main thread drains immutable messages.
// A full inbox latches a fault; it never silently evicts older IMUs.
template<class MessagePtr> class ImuInbox {
public:
    struct Entry { MessagePtr message; int64_t receive_ns; uint64_t sequence; };
    explicit ImuInbox(size_t capacity = 4096) : capacity_(capacity) {}
    void push(MessagePtr message, int64_t receive_ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (overflow_) return;
        if (queue_.size() >= capacity_) { overflow_ = true; return; }
        queue_.push_back({message, receive_ns, ++sequence_});
    }
    std::deque<Entry> drain(bool &overflow) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::deque<Entry> result;
        result.swap(queue_);
        overflow = overflow_;
        return result;
    }
private:
    const size_t capacity_;
    std::mutex mutex_;
    std::deque<Entry> queue_;
    uint64_t sequence_ = 0;
    bool overflow_ = false;
};
