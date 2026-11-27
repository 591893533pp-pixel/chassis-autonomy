#pragma once
// Scalar metadata only. Linux datagram send never waits for a collector/disk.
#include <atomic>
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>

namespace cloud_latency {
inline uint64_t now(clockid_t clock) noexcept {
  timespec ts{};
  if (clock_gettime(clock, &ts) != 0) return 0;
  return uint64_t(ts.tv_sec)*1000000000ULL + uint64_t(ts.tv_nsec);
}
class Probe {
 public:
  Probe() noexcept : fd_(socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)) {
    addr_.sun_family = AF_UNIX;
    std::strcpy(addr_.sun_path, "/dev/shm/poke-cloud-latency.sock");
  }
  ~Probe() { if (fd_ >= 0) close(fd_); }
  void emit(const char* stage, uint32_t sensor, uint64_t stamp) noexcept {
    const auto mono=now(CLOCK_MONOTONIC), utc=now(CLOCK_REALTIME);
    char buf[384];
    const int size=std::snprintf(buf, sizeof(buf), "%s,%d,%llu,%u,%llu,%llu,%llu,0,0,%llu\n",
      stage, int(getpid()), (unsigned long long)++seq_, sensor,
      (unsigned long long)stamp, (unsigned long long)mono,
      (unsigned long long)utc, (unsigned long long)dropped_.load());
    if (fd_ < 0 || size <= 0 || size >= int(sizeof(buf)) ||
        sendto(fd_, buf, size, MSG_DONTWAIT | MSG_NOSIGNAL,
               reinterpret_cast<const sockaddr*>(&addr_), sizeof(addr_)) != size)
      ++dropped_;
  }
 private:
  int fd_;
  sockaddr_un addr_{};
  std::atomic<uint64_t> seq_{0}, dropped_{0};
};
inline Probe& probe() { static Probe instance; return instance; }
}
