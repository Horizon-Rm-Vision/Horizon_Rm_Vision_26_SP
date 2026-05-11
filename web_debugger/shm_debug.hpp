#ifndef WEB_DEBUGGER__SHM_DEBUG_HPP
#define WEB_DEBUGGER__SHM_DEBUG_HPP
/**
 * shm_debug.hpp - 共享内存调试工具（单头文件）
 *
 * 功能：
 * 1. 将 cv::Mat 编码为 JPEG 写入 /dev/shm/debug_frame（共享内存模式）
 * 2. 将 JSON 数据写入 /dev/shm/cmd_log.json、serial_log.json、target_log.json
 *
 * 共享内存格式（与 web.py 约定）：
 *   [uint32_t little-endian jpg_size][jpg_bytes ...]
 *
 * 使用方法：
 *   #include "web_debugger/shm_debug.hpp"
 *
 *   // 在 main() 前创建全局对象
 *   web_debugger::ShmDebug shm_debug;
 *
 *   // 在主循环中写入帧和日志
 *   shm_debug.write_frame(img);         // 编码 img 为 JPEG 写入共享内存
 *   shm_debug.write_cmd_log(json_str);  // 写入命令日志 JSON 字符串
 */

#include <opencv2/opencv.hpp>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <nlohmann/json.hpp>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace web_debugger
{

class ShmDebug
{
public:
  static constexpr const char * kShmPath = "/dev/shm/debug_frame";
  static constexpr size_t kShmSize = 2 * 1024 * 1024;  // 2MB

  static constexpr const char * kFrameFile = "/dev/shm/debug_frame.jpg";
  static constexpr const char * kCmdLog = "/dev/shm/cmd_log.json";
  static constexpr const char * kSerialLog = "/dev/shm/serial_log.json";
  static constexpr const char * kTargetLog = "/dev/shm/target_log.json";

  ShmDebug()
  {
    init_shm();
  }

  ~ShmDebug()
  {
    if (mapped_data_ != MAP_FAILED && mapped_data_ != nullptr) {
      munmap(mapped_data_, kShmSize);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  // ---------- 写入视频帧 ----------
  // 将 cv::Mat 编码为 JPEG 并写入共享内存 + 回退文件
  bool write_frame(const cv::Mat & img, int jpeg_quality = 80)
  {
    if (img.empty()) return false;

    // 编码为 JPEG
    std::vector<uchar> jpg_buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    cv::imencode(".jpg", img, jpg_buf, params);

    if (jpg_buf.empty()) return false;

    // 方式1: 写入共享内存
    if (shm_ready_) {
      uint32_t size = static_cast<uint32_t>(jpg_buf.size());
      if (size + sizeof(uint32_t) <= kShmSize) {
        std::memcpy(mapped_data_, &size, sizeof(uint32_t));
        std::memcpy(static_cast<uint8_t *>(mapped_data_) + sizeof(uint32_t),
                    jpg_buf.data(), size);
      }
    }

    // 方式2: 写入回退文件
    std::ofstream ofs(kFrameFile, std::ios::binary | std::ios::trunc);
    if (ofs.is_open()) {
      ofs.write(reinterpret_cast<const char *>(jpg_buf.data()), jpg_buf.size());
    }

    return true;
  }

  // ---------- 写入 JSON 日志 ----------
  bool write_cmd_log(const std::string & json_str)
  {
    return write_json_file(kCmdLog, json_str);
  }

  bool write_serial_log(const std::string & json_str)
  {
    return write_json_file(kSerialLog, json_str);
  }

  bool write_target_log(const std::string & json_str)
  {
    return write_json_file(kTargetLog, json_str);
  }

  // ---------- 累积式日志（自动 append + 限长 + 写盘）----------
  void append_cmd(const std::string & key, double value, size_t max_points = 500)
  {
    auto & arr = cmd_log_[key];
    arr.push_back(value);
    trim_array(arr, max_points);
    write_cmd_log(cmd_log_.dump());
  }

  void append_serial(const std::string & key, double value, size_t max_points = 500)
  {
    auto & arr = serial_log_[key];
    arr.push_back(value);
    trim_array(arr, max_points);
    write_serial_log(serial_log_.dump());
  }

  void append_target(const std::string & key, double value, size_t max_points = 500)
  {
    auto & arr = target_log_[key];
    arr.push_back(value);
    trim_array(arr, max_points);
    write_target_log(target_log_.dump());
  }

  // 清空累积日志
  void reset_cmd_log() { cmd_log_.clear(); }
  void reset_serial_log() { serial_log_.clear(); }
  void reset_target_log() { target_log_.clear(); }

private:
  int fd_ = -1;
  void * mapped_data_ = nullptr;
  bool shm_ready_ = false;

  nlohmann::json cmd_log_;
  nlohmann::json serial_log_;
  nlohmann::json target_log_;

  static void trim_array(nlohmann::json & arr, size_t max_points)
  {
    if (arr.is_array() && arr.size() > max_points) {
      arr = nlohmann::json(arr.end() - max_points, arr.end());
    }
  }

  void init_shm()
  {
    // 打开或创建共享内存文件
    fd_ = shm_open("debug_frame", O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
      // 回退：尝试直接打开 /dev/shm/debug_frame
      fd_ = open(kShmPath, O_CREAT | O_RDWR, 0666);
    }
    if (fd_ < 0) return;

    // 确保大小足够
    struct stat st;
    if (fstat(fd_, &st) == 0 && static_cast<size_t>(st.st_size) < kShmSize) {
      (void)ftruncate(fd_, static_cast<off_t>(kShmSize));
    } else if (fstat(fd_, &st) != 0) {
      (void)ftruncate(fd_, static_cast<off_t>(kShmSize));
    }

    mapped_data_ = mmap(nullptr, kShmSize, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd_, 0);
    if (mapped_data_ == MAP_FAILED) {
      mapped_data_ = nullptr;
      return;
    }

    shm_ready_ = true;
  }

  bool write_json_file(const char * path, const std::string & json_str)
  {
    std::ofstream ofs(path, std::ios::trunc);
    if (!ofs.is_open()) return false;
    ofs << json_str;
    return true;
  }
};

}  // namespace web_debugger

#endif  // WEB_DEBUGGER__SHM_DEBUG_HPP
