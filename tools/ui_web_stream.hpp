#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tools/ui_manager.hpp"
#include "tools/ui_stream_recorder.hpp"

namespace tools {

class UIWebStream {
public:
  explicit UIWebStream(const std::string & config_path);
  UIWebStream(const std::string & host, uint16_t port, bool enabled);
  ~UIWebStream();

  bool isEnabled() const { return enabled_; }

  void beginFrame(int width, int height);
  void capturePanels(const UIManager & ui_manager);
  void sendFrame();

  /// @brief Capture raw camera image for optional transmission to receiver.
  ///        Image is JPEG-compressed in a background thread and written to
  ///        POSIX shared memory (throttled to ~30 FPS).  Only effective when
  ///        `ui.web.send_image` is true in the YAML config.
  void sendImage(const cv::Mat & img);

  // Register handler for exposure control from receiver
  void setExposureHandler(std::function<void(double)> handler);

private:
  void initSocket();
  void closeSocket();
  nlohmann::json buildJson(const UIStreamFrame & frame) const;

  // --- control channel (receiver -> local) ---
  void initControl();
  void closeControl();
  void controlLoop();
  void handleControlMessage(const std::string & msg);

  // --- shared-memory image writer ---
  void shmInit();
  void shmClose();
  void compressionLoop();

  bool enabled_{false};
  int socket_{-1};
  sockaddr_in destination_{};
  std::string host_{"127.0.0.1"};
  uint16_t port_{9876};
  uint16_t control_port_{0};

  std::vector<UIElement> left_elements_;
  std::vector<UIElement> right_elements_;
  std::string program_mode_{"default"};
  int left_y_offset_{30};
  int right_y_offset_{30};
  int line_height_{25};
  double font_scale_{0.6};
  int thickness_{2};

  // --- image transmission members ---
  bool send_image_{false};
  int image_quality_{75};
  int max_image_dim_{640};  // longest side (pixels); 0 = no resize

  // --- control channel ---
  int control_socket_{-1};
  std::thread control_thread_;
  std::atomic<bool> control_running_{false};
  std::function<void(double)> exposure_handler_;
  std::mutex control_mutex_;

  int shm_fd_{-1};
  void * shm_ptr_{nullptr};
  static constexpr size_t kShmMaxSize = 2 * 1024 * 1024;  // 2 MB

  // Background compression thread
  std::thread compress_thread_;
  std::mutex img_mutex_;
  std::condition_variable img_cv_;
  cv::Mat pending_img_;
  bool has_pending_{false};
  std::atomic<bool> compress_running_{false};
  std::chrono::steady_clock::time_point last_img_send_time_;
};

}  // namespace tools
