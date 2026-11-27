#pragma once

#include <optional>
#include <string>
#include <utility>

namespace poke {

enum class ErrorCode {
    Ok = 0,
    InvalidConfig,
    InvalidState,
    InvalidArgument,
    TransportError,
    Timeout,
    DeadlineMiss,
    FeedbackStale,
    MotorFault,
    LimitViolation,
    PlanningFailed,
    Cancelled,
    Busy,
    Unsupported,
    InternalError,
};

class Status {
public:
    Status() = default;
    Status(ErrorCode code, std::string message = {})
        : code_(code), message_(std::move(message)) {}

    static Status ok() { return {}; }
    bool isOk() const { return code_ == ErrorCode::Ok; }
    explicit operator bool() const { return isOk(); }
    ErrorCode code() const { return code_; }
    const std::string& message() const { return message_; }

private:
    ErrorCode code_{ErrorCode::Ok};
    std::string message_;
};

template <class T>
class Result {
public:
    Result(T value) : value_(std::move(value)) {}
    Result(Status status) : status_(std::move(status)) {}

    bool isOk() const { return status_.isOk() && value_.has_value(); }
    explicit operator bool() const { return isOk(); }
    const Status& status() const { return status_; }
    T& value() { return *value_; }
    const T& value() const { return *value_; }
    T takeValue() { return std::move(*value_); }

private:
    Status status_{};
    std::optional<T> value_;
};

}  // namespace poke
