#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include <map>
#include <mutex>
#include <sstream>
#include <cstring>

// Report packet sync flags, not ROS receive timestamps. Callback state is per lidar.
class PacketTimeMonitor {
  struct State { int type; uint64_t stamp; int dev; double seen; };
  std::mutex mutex_;
  std::map<uint32_t, State> states_;
  uint16_t observer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr device_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static void receive(uint32_t handle, const uint8_t dev, LivoxLidarEthernetPacket* p, void* ctx) {
    if (!p || p->data_type == kLivoxLidarImuData) return;
    auto* self = static_cast<PacketTimeMonitor*>(ctx);
    uint64_t stamp=0; std::memcpy(&stamp,p->timestamp,sizeof(stamp));
    std::lock_guard<std::mutex> guard(self->mutex_);
    self->states_[handle] = {p->time_type, stamp, dev, steady()};
  }
 public:
  static void device_info(uint32_t handle, const uint8_t dev, const char* info, void* ctx) {
    if (!info || !ctx) return;
    auto* self = static_cast<PacketTimeMonitor*>(ctx);
    std_msgs::msg::String msg;
    // Preserve SDK JSON verbatim: absent diagnostics must not become zero offsets.
    msg.data = "{\"ip\":\"" + livox_ros::IpNumToString(handle) +
      "\",\"device_type\":" + std::to_string(dev) + ",\"info\":" + info + "}";
    self->device_pub_->publish(msg);
  }
  explicit PacketTimeMonitor(rclcpp::Node* node) {
    pub_=node->create_publisher<std_msgs::msg::String>("/livox/time_sync",10);
    device_pub_=node->create_publisher<std_msgs::msg::String>("/livox/device_diagnostics",10);
    SetLivoxLidarInfoCallback(device_info,this);
    observer_=LivoxLidarAddPointCloudObserver(receive,this);
    timer_=node->create_wall_timer(std::chrono::milliseconds(200),[this]() {
      std::ostringstream out; out << "{"; bool first=true;
      { std::lock_guard<std::mutex> guard(mutex_);
        for (const auto& item: states_) {
          if (!first) { out << ","; }
          first=false;
          const auto& s=item.second;
          out << "\"" << livox_ros::IpNumToString(item.first) << "\":{\"time_type\":" << s.type
              << ",\"packet_timestamp_ns\":" << s.stamp << ",\"device_type\":" << s.dev
              << ",\"packet_age_ms\":" << (steady()-s.seen)*1000 << "}";
        }
      }
      out << "}"; std_msgs::msg::String msg; msg.data=out.str(); pub_->publish(msg);
    });
  }
  ~PacketTimeMonitor() { SetLivoxLidarInfoCallback(nullptr,nullptr); timer_->cancel(); LivoxLidarRemovePointCloudObserver(observer_); }
};
