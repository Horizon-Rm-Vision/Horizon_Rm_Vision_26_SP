#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
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
  uint8_t mode;               // 一字节 mode(高位携带己方颜色: mode/4, 低2位为原模式: mode%4)
  uint8_t bullet_speed;       // 一字节弹速，单位为0.1m/s，接收时乘10转换为整数发送
  #ifdef SR_VEL // 添加云台前馈角速度数据收发
    float pitch_vel;
    float yaw_vel;
  #endif
  #ifdef SENTRY_SR
  uint8_t game_progress;           // 比赛阶段
  uint16_t stage_remain_time;                 // 剩余时间
  uint16_t current_hp;                // 当前血量
  uint16_t ally_outpost_hp;                // 己方基地血量
  uint8_t state;                // 姿态
  uint8_t energy_state;             // 能量机关状态
  uint16_t bullets;                // 子弹数量
  uint8_t judge;                 //决策
  #endif
  uint8_t tail = 0xDC;        // 包尾 0xDC
};

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head = 0xCD;        // 包头 0xCD
  float pitch;                // 四字节 pitch
  float yaw;                  // 四字节 yaw
  uint8_t mode;               // 一字节 mode
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
  int8_t gimbal;               // 云台编号，预留字段，暂时填0
  #endif
  uint8_t tail = 0xDC;        // 包尾 0xDC
};

static_assert(sizeof(VisionToGimbal) <= 64);

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
  int8_t self_color = -1;
  #ifdef SENTRY_SR
  uint8_t game_progress;           // 比赛阶段
  uint16_t stage_remain_time;                 // 剩余时间
  uint16_t current_hp;                 // 血量
  uint16_t ally_outpost_hp;                // 己方基地血量 
  uint8_t state;                // 姿态
  uint8_t energy_state;             // 能量机关状态
  uint16_t bullets;                // 子弹数量
  uint8_t judge;                  //决策
   #endif
};

int8_t latest_self_color();

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
    float pitch_acc, float vx, float vy, float wz,int form,int gimbal);
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

  bool open_serial();
  void configure_serial();
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP