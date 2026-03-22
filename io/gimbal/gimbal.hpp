#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <iomanip>
#include <sstream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head = 0xCD;        // 包头 0xCD
  float pitch;                // 四字节 pitch
  float yaw;                  // 四字节 yaw
  uint8_t mode;               // 一字节 mode
  // uint32_t timestamp;         // 四字节时间戳
  uint32_t seq_num;           // 四字节序列号，用于延迟测量
  uint8_t bullet_speed;       // 一字节弹速
  #ifdef SR_VEL // 添加云台前馈角速度数据收发
    // float yaw_vel;
    // float pitch_vel;
  #endif
  #ifdef SENTRY_SR
  uint8_t game_status;           // 比赛阶段
  uint16_t blood;                 // 血量
  uint16_t bullet;                // 弹量
  // bool superpower;             // 超电开关
  #endif
  uint8_t tail = 0xDC;        // 包尾 0xDC
};

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head = 0xCD;        // 包头 0xCD
  float pitch;                // 四字节 pitch
  float yaw;                  // 四字节 yaw
  uint8_t mode;               // 一字节 mode
  // uint32_t timestamp;         // 四字节时间戳
  uint32_t seq_num;           // 四字节序列号，用于延迟测量
  #ifdef SR_VEL // 添加云台前馈角速度数据收发
  float pitch_vel;
  float yaw_vel;
  // float pitch_acc;
  // float yaw_acc;
  #endif
  #ifdef SENTRY_SR
  float vx;                   // x方向速度
  float vy;                   // y方向速度
  float wz;                   // 角速度
  int8_t form;                // 哨兵姿态
  #endif
  uint8_t tail = 0xDC;        // 包尾 0xDC
};

static_assert(sizeof(VisionToGimbal) <= 64);

// enum class GimbalMode
// {
//   IDLE,        // 空闲
//   AUTO_AIM,    // 自瞄
//   SMALL_BUFF,  // 小符
//   BIG_BUFF     // 大符
// };

enum class GimbalMode
{
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF,    // 大符
  IDLE         // 空闲
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  #ifdef SENTRY_SR
  uint8_t game_status;           // 比赛阶段
  uint16_t blood;                 // 血量
  uint16_t bullet;                // 弹量
  #endif
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);
  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point t);
  #ifdef NOVA_Q
  int q_size() const;
  #endif

  #ifndef SENTRY_SR
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);
  #endif
  
  #ifdef SENTRY_SR
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc, float vx, float vy, float wz,int form);
  #endif

  void send(io::VisionToGimbal VisionToGimbal);

  // 数据包转换为十六进制字符串调试
  std::string packet_to_hex(const void* data, size_t size) const;

private:
  int fd_ = -1;
  std::string com_port_;
  
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  //GimbalMode mode_ = GimbalMode::AUTO_AIM;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  // 通信延迟测量相关
  std::atomic<uint32_t> seq_counter_{0};
  std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> send_times_;
  std::mutex send_times_mutex_;

  bool open_serial();
  void configure_serial();
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP