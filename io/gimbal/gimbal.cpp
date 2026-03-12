#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iomanip>

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  com_port_ = tools::read<std::string>(yaml, "com_port");

  if (com_port_ == "auto") {
    // 候选串口列表
    const char* candidates[] = {
        "/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyTHS0","/dev/ttyCH341USB0",
        "/dev/ttyACM1", "/dev/ttyUSB1", "/dev/ttyTHS1","/dev/ttyCH341USB1",
        "/dev/ttyACM2", "/dev/ttyUSB2", "/dev/ttyTHS2","/dev/ttyCH341USB2",
        "/dev/ttyACM3", "/dev/ttyUSB3", "/dev/ttyTHS3","/dev/ttyCH341USB3",
        "/dev/ttyACM4", "/dev/ttyUSB4", "/dev/ttyTHS4","/dev/ttyCH341USB4",
        nullptr
    };
    bool found = false;
    for (const char** p = candidates; *p != nullptr; ++p) {
      com_port_ = *p;  // 临时设置为当前候选端口
      if (open_serial()) {
        tools::logger()->info("[Gimbal] Auto selected port: {}", com_port_);
        found = true;
        break;
      }
    }
    if (!found) {
      tools::logger()->error("[Gimbal] No valid serial port found in auto mode.");
      exit(1);
    }
  } else {
    tools::logger()->info("[Gimbal] Initializing gimbal communication on port: {}", com_port_);
    if (!open_serial()) {
      tools::logger()->error("[Gimbal] Failed to open serial port: {}", com_port_);
      exit(1);
    }
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  // Wait for first data
  queue_.pop();
  tools::logger()->info("[Gimbal] First quaternion data received.");
}

Gimbal::~Gimbal()
{
  tools::logger()->info("[Gimbal] Shutting down gimbal communication");
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (fd_ >= 0) {
    close(fd_);
    tools::logger()->info("[Gimbal] Serial port closed");
  }
}

bool Gimbal::open_serial()
{
  tools::logger()->info("[Gimbal] Opening serial port: {}", com_port_);
  fd_ = open(com_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    tools::logger()->error("[Gimbal] Failed to open serial port {}: {}", com_port_, strerror(errno));
    return false;
  }
  
  tools::logger()->info("[Gimbal] Successfully opened serial port, fd: {}", fd_);
  configure_serial();
  return true;
}

void Gimbal::configure_serial()
{
  tools::logger()->info("[Gimbal] Configuring serial port parameters");
  struct termios options;
  tcgetattr(fd_, &options);
  
  // Set baud rate
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  
  // Enable receiver and set local mode
  options.c_cflag |= (CLOCAL | CREAD);
  
  // Set data bits, stop bits, parity
  options.c_cflag &= ~PARENB;  // No parity
  options.c_cflag &= ~CSTOPB;  // 1 stop bit
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;      // 8 data bits
  
  // Disable software flow control
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL);
  
  // Raw input
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP);
  
  // Raw output
  options.c_oflag &= ~OPOST;
  
  // Set timeouts - return immediately with available data
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  
  tcsetattr(fd_, TCSANOW, &options);
  tcflush(fd_, TCIOFLUSH);
  
  tools::logger()->info("[Gimbal] Serial port configured: 115200 baud, 8N1, no flow control");
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

// std::string Gimbal::str(GimbalMode mode) const
// {
//   switch (mode) {
//     case GimbalMode::IDLE:
//       return "IDLE";
//     case GimbalMode::AUTO_AIM:
//       return "AUTO_AIM";
//     case GimbalMode::SMALL_BUFF:
//       return "SMALL_BUFF";
//     case GimbalMode::BIG_BUFF:
//       return "BIG_BUFF";
//     default:
//       return "INVALID";
//   }
// }

//转换GimbalMode数值为对应的字符串
std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    case GimbalMode::IDLE:
      return "IDLE";
    default:
      return "INVALID";
  }
}

#ifndef NOVA_Q
Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}
#endif

#ifdef NOVA_Q
Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [front, back] = queue_.peek2();
    auto [q_a, t_a] = front;
    auto [q_b, t_b] = back;

    if (t <= t_a) {
      return q_a;
    }

    if (t_a < t && t <= t_b) {
      double t_ab = tools::delta_time(t_a, t_b);
      double t_ac = tools::delta_time(t_a, t);
      double k = t_ac / t_ab;
      return q_a.slerp(k, q_b).normalized();
    }

    queue_.pop();
  }
}

int Gimbal::q_size() const
{
  return queue_.empty() ? 0 : 1 + queue_.size();
}
#endif

// 仅用于capture
Eigen::Quaterniond Gimbal::imu_at(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

std::string Gimbal::packet_to_hex(const void* data, size_t size) const
{
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (size_t i = 0; i < size; ++i) {
    ss << std::setw(2) << static_cast<int>(bytes[i]);
    if (i < size - 1) ss << " ";
  }
  return ss.str();
}

//仅用于fire_test.cpp
void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // 复制数据到局部变量以避免packed结构体引用问题
  uint8_t mode = VisionToGimbal.mode;
  float yaw = VisionToGimbal.yaw;
  float pitch = VisionToGimbal.pitch;
  #ifdef SR_VEL
    float yaw_vel = VisionToGimbal.yaw_vel;
    //float yaw_acc = VisionToGimbal.yaw_acc;
    float pitch_vel = VisionToGimbal.pitch_vel;
    //float pitch_acc = VisionToGimbal.pitch_acc;
  #endif
  
  // 赋值给tx_data_
  tx_data_.mode = mode;
  tx_data_.yaw = yaw;
  tx_data_.pitch = pitch;
  tx_data_.timestamp = 0;  // 时间戳暂时填0
  #ifdef SR_VEL
    tx_data_.yaw_vel = yaw_vel;
    //tx_data_.yaw_acc = yaw_acc;
    tx_data_.pitch_vel = pitch_vel;
    //tx_data_.pitch_acc = pitch_acc;
  #endif
  
  if (fd_ < 0) {
    tools::logger()->error("[Gimbal] Cannot send data - serial port not open");
    return;
  }
  
  // 使用局部变量记录发送的数据内容
  tools::logger()->debug("[Gimbal] Sending data - Mode: {}, Pitch: {:.3f}, Yaw: {:.3f}",
                        mode, pitch, yaw);
  
  ssize_t bytes_written = write(fd_, &tx_data_, sizeof(tx_data_));
  if (bytes_written != sizeof(tx_data_)) {
    tools::logger()->warn("[Gimbal] Failed to write serial, expected {} bytes, got {} bytes, error: {}",
                         sizeof(tx_data_), bytes_written, strerror(errno));
  } else {
    tools::logger()->debug("[Gimbal] Successfully sent {} bytes to gimbal", bytes_written);
  }
}

#ifndef SENTRY_SR
// 自瞄向电控发送数据(普通模式)
void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  uint8_t mode;
  if (control) 
  {
      if (fire) 
      {
          mode = 57;  // 控制且开火，对应NT_M6的111001（十六进制原始数据39）
      } 
      else 
      {
          mode = 49;  // 控制但不开火，对应NT_M6的110001（十六进制原始数据31）
      }
  } 
  else 
  {
      mode = 1;      // 不控制，对应NT_M6的自瞄模式默认标志位001（十六进制原始数据01）
  }

  tx_data_.mode = mode;  
  // p/y值赋给tx_data_，自瞄原始数据是弧度制，需要转换为角度制发送
  tx_data_.yaw = -yaw * (180.0 / M_PI);  // 弧度转换为角度并取负
  tx_data_.pitch = -pitch * (180.0 / M_PI);  // 弧度转换为角度并取负
  tx_data_.timestamp = 0;  // 时间戳暂时填0
  #ifdef SR_VEL
    tx_data_.yaw_vel = -yaw_vel* (180.0 / M_PI);  // 角速度转换为角度每秒并取负
    tx_data_.pitch_vel = -pitch_vel* (180.0 / M_PI);  // 角速度转换为角度每秒并取负
    //tx_data_.yaw_acc = -yaw_acc* (180.0 / M_PI);  // 角加速度转换为角度每秒平方并取负
    //tx_data_.pitch_acc = -pitch_acc* (180.0 / M_PI);  // 角加速度转换为角度每秒平方并取负
  #endif
  
  if (fd_ < 0) {
    tools::logger()->error("[Gimbal] Cannot send data - serial port not open");
    return;
  }
  
  // 使用局部变量记录发送的数据内容
  std::string mode_str = control ? (fire ? "CONTROL_FIRE" : "CONTROL_NO_FIRE") : "NO_CONTROL";
  tools::logger()->debug("[Gimbal] Sending data - Mode: {} ({}), Pitch: {:.3f}, Yaw: {:.3f}",
                        mode_str, mode, -pitch * (180.0 / M_PI), -yaw * (180.0 / M_PI));
  
  ssize_t bytes_written = write(fd_, &tx_data_, sizeof(tx_data_));
  if (bytes_written != sizeof(tx_data_)) {
    tools::logger()->warn("[Gimbal] Failed to write serial, expected {} bytes, got {} bytes, error: {}",
                         sizeof(tx_data_), bytes_written, strerror(errno));
  } else {
    tools::logger()->debug("[Gimbal] Successfully sent {} bytes to gimbal", bytes_written);
    
    // // 记录发送原始数据
    // tools::logger()->trace("[Gimbal] Raw TX data: {}", 
    //                       packet_to_hex(&tx_data_, sizeof(tx_data_)));

  }
}
#endif
#ifdef SENTRY_SR
// 自瞄向电控发送数据(哨兵模式，带导航通信内容)
void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc,float vx, float vy, float wz)
{
  uint8_t mode;
  if (control) 
  {
      if (fire) 
      {
          mode = 57;  // 控制且开火，对应NT_M6的111001（十六进制原始数据39）
      } 
      else 
      {
          mode = 49;  // 控制但不开火，对应NT_M6的110001（十六进制原始数据31）
      }
  } 
  else 
  {
      mode = 1;      // 不控制，对应NT_M6的自瞄模式默认标志位001（十六进制原始数据01）
  }

  tx_data_.mode = mode;  
  // p/y值赋给tx_data_，自瞄原始数据是弧度制，需要转换为角度制发送
  tx_data_.yaw = -yaw * (180.0 / M_PI);  // 弧度转换为角度并取负
  tx_data_.pitch = -pitch * (180.0 / M_PI);  // 弧度转换为角度并取负
  tx_data_.timestamp = 0;  // 时间戳暂时填0
  tx_data_.vx = vx;
  tx_data_.vy = vy;
  tx_data_.wz = wz;
  //哨兵导航
  
  #ifdef SR_VEL
    tx_data_.yaw_vel = -yaw_vel* (180.0 / M_PI);  // 角速度转换为角度每秒并取负
    tx_data_.pitch_vel = -pitch_vel* (180.0 / M_PI);  // 角速度转换为角度每秒并取负
    //tx_data_.yaw_acc = -yaw_acc* (180.0 / M_PI);  // 角加速度转换为角度每秒平方并取负
    //tx_data_.pitch_acc = -pitch_acc* (180.0 / M_PI);  // 角加速度转换为角度每秒平方并取负
  #endif
  
  if (fd_ < 0) {
    tools::logger()->error("[Gimbal] Cannot send data - serial port not open");
    return;
  }
  
  // 使用局部变量记录发送的数据内容
  std::string mode_str = control ? (fire ? "CONTROL_FIRE" : "CONTROL_NO_FIRE") : "NO_CONTROL";
  tools::logger()->debug("[Gimbal] Sending data - Mode: {} ({}), Pitch: {:.3f}, Yaw: {:.3f}",
                        mode_str, mode, -pitch * (180.0 / M_PI), -yaw * (180.0 / M_PI));
  
  ssize_t bytes_written = write(fd_, &tx_data_, sizeof(tx_data_));
  if (bytes_written != sizeof(tx_data_)) {
    tools::logger()->warn("[Gimbal] Failed to write serial, expected {} bytes, got {} bytes, error: {}",
                         sizeof(tx_data_), bytes_written, strerror(errno));
  } else {
    tools::logger()->debug("[Gimbal] Successfully sent {} bytes to gimbal", bytes_written);
    
    // // 记录发送原始数据
    // tools::logger()->trace("[Gimbal] Raw TX data: {}", 
    //                       packet_to_hex(&tx_data_, sizeof(tx_data_)));

  }
}
#endif

// 自瞄从电控读取数据
void Gimbal::read_thread()
{
  // 统计接收fps,用不到时注释上
  auto fps_start = std::chrono::steady_clock::now();
  int fps_count = 0;
  
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  const size_t packet_size = sizeof(GimbalToVision);
  
  uint8_t buffer[4096];  
  ssize_t bytes_read;
  size_t data_index = 0;
  
  while (!quit_) {
    if (error_count > 5000) {
      tools::logger()->warn("[Gimbal] Too many errors ({}), attempting to reconnect...", error_count);  // 先log
      error_count = 0;  
      reconnect();
      continue;
    }
    
    if (fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // 防止 data_index 接近 buffer 上限时无法 read
    if (data_index >= sizeof(buffer) - packet_size) {
      tools::logger()->warn("[Gimbal] Buffer nearly full ({}), discarding all data.", data_index);
      data_index = 0;
      error_count++;
      continue;
    }
    
    bytes_read = read(fd_, buffer + data_index, sizeof(buffer) - data_index);
    if (bytes_read < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        error_count++;
        tools::logger()->debug("[Gimbal] Read error: {} (errno: {})", strerror(errno), errno);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    
    if (bytes_read == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    
    // 记录原始接收数据
    tools::logger()->trace("[Gimbal] Received {} bytes raw data: {}", 
                          bytes_read, packet_to_hex(buffer + data_index, bytes_read));
    
    data_index += bytes_read;
    
    // 一次处理完 buffer 里所有完整包
    size_t i = 0;
    while (i + packet_size <= data_index) {
      // 找包头
      if (buffer[i] != 0xCD) {
        i++;
        continue;
      }
      
      // 校验包尾
      if (buffer[i + packet_size - 1] != 0xDC) {
        tools::logger()->warn("[Gimbal] Packet tail mismatch at offset {}, expected 0xDC, got 0x{:02x}",
                             i, buffer[i + packet_size - 1]);
        i++;
        continue;
      }
      
      auto t = std::chrono::steady_clock::now();
      // Copy valid packet
      std::memcpy(&rx_data_, buffer + i, packet_size);
      
      // 记录原始数据包
      tools::logger()->debug("[Gimbal] Found complete packet at offset {}, raw: {}",
                            i, packet_to_hex(buffer + i, packet_size));
      
      // 复制到局部变量以避免packed结构体引用问题
      uint8_t mode = rx_data_.mode;
      float yaw   = -rx_data_.yaw   * (M_PI / 180.0f); // 接收时从角度制转换为弧度制并取负
      float pitch = -rx_data_.pitch * (M_PI / 180.0f); // 接收时从角度制转换为弧度制并取负
      uint8_t bullet_speed = rx_data_.bullet_speed;
      #ifdef SR_VEL
        float yaw_vel = -rx_data_.yaw_vel * (M_PI / 180.0);  // 接收时从角度每秒转换为弧度每秒并取负
        float pitch_vel = -rx_data_.pitch_vel * (M_PI / 180.0);  // 接收时从角度每秒转换为弧度每秒并取负
        //float yaw_acc = -rx_data_.yaw_acc * (M_PI / 180.0);  // 接收时从角度每秒平方转换为弧度每秒平方并取负
        //float pitch_acc = -rx_data_.pitch_acc * (M_PI / 180.0);  // 接收时从角度每秒平方转换为弧度每秒平方并取负
      #endif
      #ifdef SENTRY_SR
      //哨兵导航相关数据
      uint8_t game_status = rx_data_.game_status;
      uint8_t blood = rx_data_.blood;
      uint8_t bullet = rx_data_.bullet;
      #endif
      // 使用yaw和pitch计算四元数（roll设为0）
      //单位转换
      //double d2r = M_PI / 180.0;
      Eigen::AngleAxisd yaw_angle  (yaw,   Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd roll_angle (0.0,   Eigen::Vector3d::UnitX());
      
      Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
      q.normalize();
      
      if (mode <= 3) {
        queue_.push({q, t});
        
        //fps统计,用不到时注释上
        fps_count++;
        auto fps_now = std::chrono::steady_clock::now();
        std::chrono::duration<double> fps_elapsed = fps_now - fps_start;
        if (fps_elapsed.count() >= 1.0) {
          tools::logger()->warn("[Gimbal] push fps: {}", fps_count);
          fps_count = 0;
          fps_start = fps_now;
        }
        
        //传入数值给state_，供外部调用
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.yaw = yaw;
            state_.pitch = pitch;
            state_.bullet_speed = static_cast<float>(bullet_speed);
            #ifdef SR_VEL
              state_.yaw_vel = yaw_vel;
              state_.pitch_vel = pitch_vel;
              //state_.yaw_acc = yaw_acc;
              //state_.pitch_acc = pitch_acc;
            #endif
            #ifndef SR_VEL
            state_.yaw_vel = 0.0f;  // 速度暂时填0
            state_.pitch_vel = 0.0f;  // 速度暂时填0
            #endif
            state_.bullet_count = 0;  // 子弹计数暂时填0
            #ifdef SENTRY_SR
            //哨兵导航相关数据
            state_.game_status = game_status;
            state_.blood = blood;
            state_.bullet = bullet;
            #endif
        }
            //本次接收前后模式变化记录日志
            GimbalMode old_mode = mode_;
            switch (mode) {
              case 0:
                mode_ = GimbalMode::AUTO_AIM;
                break;
              case 1:
                mode_ = GimbalMode::SMALL_BUFF;
                break;
              case 2:
                mode_ = GimbalMode::BIG_BUFF;
                break;
              case 3:
                mode_ = GimbalMode::IDLE;
                break;
              default:
                mode_ = GimbalMode::IDLE;
                break;
            }
        
            // 使用局部变量记录解析后的数据内容
            tools::logger()->info("[Gimbal] Parsed read data - Mode: {}->{}, Pitch: {:.3f}, Yaw: {:.3f}, "
                                 "BulletSpeed: {}, Quaternion: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                                 str(old_mode), str(mode_), -pitch * (180.0 / M_PI), -yaw * (180.0 / M_PI), bullet_speed, 
                                 q.w(), q.x(), q.y(), q.z());
        
        error_count = 0;
      } else {
        tools::logger()->warn("[Gimbal] Skipping packet with invalid mode: {}, raw data: {}",
                             mode, packet_to_hex(buffer + i, packet_size));
      }
      
      i += packet_size;  // 无论 mode 是否有效，跳过这个包
    }
    
    // 把未处理的残留数据移到 buffer 头部
    size_t remaining = data_index - i;
    if (remaining > 0 && i > 0) {
      std::memmove(buffer, buffer + i, remaining);
    }
    data_index = remaining;
  }
  
  tools::logger()->info("[Gimbal] read_thread stopped.");
}
//失败重连
void Gimbal::reconnect()
{
  tools::logger()->info("[Gimbal] Attempting to reconnect to serial port");
  
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
    tools::logger()->info("[Gimbal] Closed existing serial connection");
  }
  
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  if (!open_serial()) {
    tools::logger()->error("[Gimbal] Reconnect failed for port: {}", com_port_);
  } else {
    tools::logger()->info("[Gimbal] Reconnected successfully to port: {}", com_port_);
    // Clear buffer and reset state
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = GimbalMode::IDLE;
    state_ = GimbalState{};
  }
}

}  // namespace io