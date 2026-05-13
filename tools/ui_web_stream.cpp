#include "ui_web_stream.hpp"

#include <array>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include "yaml.hpp"

namespace tools {

// -----------------------------------------------------------------------
// Construction / destruction
// -----------------------------------------------------------------------

UIWebStream::UIWebStream(const std::string & config_path)
{
  try {
    YAML::Node config = YAML::LoadFile(config_path);
    if (config["ui"] && config["ui"]["web"]) {
      auto web = config["ui"]["web"];
      if (web["enabled"]) {
        enabled_ = web["enabled"].as<bool>();
      }
      if (web["host"]) {
        host_ = web["host"].as<std::string>();
      }
      if (web["port"]) {
        port_ = static_cast<uint16_t>(web["port"].as<int>());
      }
      if (web["control_port"]) {
        control_port_ = static_cast<uint16_t>(web["control_port"].as<int>());
      }
      // Image transmission settings (wust_vision-style shared memory)
      if (web["send_image"]) {
        send_image_ = web["send_image"].as<bool>();
      }
      if (web["image_quality"]) {
        int q = web["image_quality"].as<int>();
        if (q >= 1 && q <= 100) image_quality_ = q;
      }
      if (web["max_image_dim"]) {
        int d = web["max_image_dim"].as<int>();
        if (d >= 0) max_image_dim_ = d;
      }
    }
  } catch (const std::exception &) {
    enabled_ = false;
  }

  if (enabled_) {
    initSocket();
  }
}

UIWebStream::UIWebStream(const std::string & host, uint16_t port, bool enabled)
  : enabled_(enabled), host_(host), port_(port)
{
  if (enabled_) {
    initSocket();
  }
}

UIWebStream::~UIWebStream()
{
  closeSocket();
}

// -----------------------------------------------------------------------
// Socket init / close
// -----------------------------------------------------------------------

void UIWebStream::initSocket()
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port_);
  destination_.sin_addr.s_addr = ::inet_addr(host_.c_str());

  if (control_port_ == 0) {
    control_port_ = static_cast<uint16_t>(port_ + 1);
  }

  UIStreamRecorder::instance().setEnabled(true);
  UIManager::setGlobalCaptureEnabled(true);

  // Start shared-memory image writer if enabled
  if (send_image_) {
    shmInit();
  }

  initControl();
}

void UIWebStream::closeSocket()
{
  closeControl();

  // Stop compression thread first
  if (compress_running_.exchange(false)) {
    img_cv_.notify_all();
    if (compress_thread_.joinable()) {
      compress_thread_.join();
    }
  }

  // Close SHM
  shmClose();

  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

void UIWebStream::beginFrame(int width, int height)
{
  if (!enabled_) return;
  UIStreamRecorder::instance().setEnabled(true);
  UIManager::setGlobalCaptureEnabled(true);
  UIStreamRecorder::instance().resetFrame(width, height);
}

void UIWebStream::capturePanels(const UIManager & ui_manager)
{
  if (!enabled_) return;
  left_elements_ = ui_manager.leftElements();
  right_elements_ = ui_manager.rightElements();
  program_mode_ = ui_manager.programMode();
  left_y_offset_ = ui_manager.leftYOffset();
  right_y_offset_ = ui_manager.rightYOffset();
  line_height_ = ui_manager.lineHeight();
  font_scale_ = ui_manager.fontScale();
  thickness_ = ui_manager.thickness();
}

// -----------------------------------------------------------------------
// Image transmission  (wust_vision-style: SHM + background compression)
// -----------------------------------------------------------------------

void UIWebStream::shmInit()
{
  constexpr mode_t kMode = 0666;

  shm_fd_ = ::shm_open("/nova_cam_frame", O_CREAT | O_RDWR, kMode);
  if (shm_fd_ == -1) {
    send_image_ = false;
    return;
  }

  // Set /dev/shm/nova_cam_frame permissions so a non-root receiver can read
  ::fchmod(shm_fd_, kMode);

  if (::ftruncate(shm_fd_, static_cast<off_t>(kShmMaxSize)) == -1) {
    ::close(shm_fd_);
    shm_fd_ = -1;
    send_image_ = false;
    return;
  }

  shm_ptr_ = ::mmap(nullptr, kShmMaxSize, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
  if (shm_ptr_ == MAP_FAILED) {
    ::close(shm_fd_);
    shm_fd_ = -1;
    shm_ptr_ = nullptr;
    send_image_ = false;
    return;
  }

  // Start background compression thread
  compress_running_ = true;
  compress_thread_ = std::thread(&UIWebStream::compressionLoop, this);
}

void UIWebStream::shmClose()
{
  if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
    ::munmap(shm_ptr_, kShmMaxSize);
    shm_ptr_ = nullptr;
  }
  if (shm_fd_ >= 0) {
    ::close(shm_fd_);
    shm_fd_ = -1;
  }
}

void UIWebStream::sendImage(const cv::Mat & img)
{
  if (!send_image_ || img.empty()) return;

  // Throttle sender-side to ~30 FPS to avoid burning CPU on frames the
  // receiver will never see.
  auto now = std::chrono::steady_clock::now();
  if (now - last_img_send_time_ < std::chrono::milliseconds(30)) {
    return;
  }
  last_img_send_time_ = now;

  // Deep-copy the raw frame for the background compression thread.
  // We clone here so the caller is free to draw on `img` afterwards.
  cv::Mat frame = img.clone();

  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    pending_img_ = std::move(frame);
    has_pending_ = true;
  }
  img_cv_.notify_one();
}

void UIWebStream::setExposureHandler(std::function<void(double)> handler)
{
  std::lock_guard<std::mutex> lock(control_mutex_);
  exposure_handler_ = std::move(handler);
}

void UIWebStream::initControl()
{
  control_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (control_socket_ < 0) {
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = ::htons(control_port_);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (::bind(control_socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(control_socket_);
    control_socket_ = -1;
    return;
  }

  control_running_ = true;
  control_thread_ = std::thread(&UIWebStream::controlLoop, this);
}

void UIWebStream::closeControl()
{
  if (control_running_.exchange(false)) {
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }

  if (control_socket_ >= 0) {
    ::close(control_socket_);
    control_socket_ = -1;
  }
}

void UIWebStream::controlLoop()
{
  std::array<char, 1024> buf{};

  while (control_running_) {
    pollfd pfd{};
    pfd.fd = control_socket_;
    pfd.events = POLLIN;

    int ret = ::poll(&pfd, 1, 200);
    if (ret <= 0) {
      continue;
    }

    if ((pfd.revents & POLLIN) == 0) {
      continue;
    }

    sockaddr_in src{};
    socklen_t len = sizeof(src);
    ssize_t n = ::recvfrom(control_socket_, buf.data(), buf.size() - 1, 0,
                           reinterpret_cast<sockaddr *>(&src), &len);
    if (n <= 0) {
      continue;
    }

    buf[static_cast<size_t>(n)] = '\0';
    handleControlMessage(std::string(buf.data(), static_cast<size_t>(n)));
  }
}

void UIWebStream::handleControlMessage(const std::string & msg)
{
  try {
    auto payload = nlohmann::json::parse(msg);
    if (payload.value("type", "") != "control") {
      return;
    }

    const auto name = payload.value("name", "");
    if (name != "exposure_ms") {
      return;
    }

    const double value = payload.value("value", -1.0);
    if (value <= 0.0) {
      return;
    }

    std::function<void(double)> handler;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      handler = exposure_handler_;
    }
    if (handler) {
      handler(value);
    }
  } catch (const std::exception &) {
    return;
  }
}

void UIWebStream::compressionLoop()
{
  while (compress_running_) {
    cv::Mat frame;

    {
      std::unique_lock<std::mutex> lock(img_mutex_);
      img_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return has_pending_ || !compress_running_;
      });
      if (!compress_running_) break;
      if (!has_pending_) continue;
      frame = pending_img_;
      has_pending_ = false;
    }

    if (frame.empty()) continue;

    // Optional resize to keep UDP payload below ~64 KB limit
    cv::Mat send_frame = frame;
    if (max_image_dim_ > 0) {
      int h = send_frame.rows, w = send_frame.cols;
      if (std::max(w, h) > max_image_dim_) {
        double scale = static_cast<double>(max_image_dim_) / std::max(w, h);
        cv::resize(send_frame, send_frame, {}, scale, scale, cv::INTER_LINEAR);
      }
    }

    // JPEG compression (same approach as wust_vision ShmWriter)
    static const std::vector<int> jpeg_params = {
      cv::IMWRITE_JPEG_QUALITY, image_quality_
    };
    std::vector<uchar> buf;
    cv::imencode(".jpg", send_frame, buf, jpeg_params);

    // ── Write to POSIX shared memory (local consumers) ──
    if (buf.size() + 4 <= kShmMaxSize && shm_ptr_) {
      uint32_t size = static_cast<uint32_t>(buf.size());
      std::memcpy(shm_ptr_, &size, 4);
      std::memcpy(static_cast<char *>(shm_ptr_) + 4, buf.data(), size);
    }

    // ── Send via UDP with "NVIC" magic prefix (remote receiver) ──
    if (socket_ >= 0 && buf.size() + 8 <= 60000) {
      // Pack: [NVIC][4-byte size][JPEG data]
      std::vector<uint8_t> pkt(8 + buf.size());
      std::memcpy(pkt.data(), "NVIC", 4);
      uint32_t sz = static_cast<uint32_t>(buf.size());
      std::memcpy(pkt.data() + 4, &sz, 4);
      std::memcpy(pkt.data() + 8, buf.data(), buf.size());

      ::sendto(socket_, pkt.data(), pkt.size(), 0,
               reinterpret_cast<sockaddr *>(&destination_), sizeof(destination_));
    }
  }
}

nlohmann::json UIWebStream::buildJson(const UIStreamFrame & frame) const
{
  nlohmann::json payload;
  payload["type"] = "ui_frame";
  payload["frame_id"] = frame.frame_id;
  payload["width"] = frame.width;
  payload["height"] = frame.height;
  payload["program_mode"] = program_mode_;

  payload["layout"] = {
    {"left_y_offset", left_y_offset_},
    {"right_y_offset", right_y_offset_},
    {"line_height", line_height_},
    {"font_scale", font_scale_},
    {"thickness", thickness_},
  };

  payload["left"] = nlohmann::json::array();
  for (const auto & elem : left_elements_) {
    if (!elem.enabled) continue;
    payload["left"].push_back({
      {"key", elem.key},
      {"text", elem.text},
      {"color", {elem.color[0], elem.color[1], elem.color[2]}}
    });
  }

  payload["right"] = nlohmann::json::array();
  for (const auto & elem : right_elements_) {
    if (!elem.enabled) continue;
    payload["right"].push_back({
      {"key", elem.key},
      {"text", elem.text},
      {"color", {elem.color[0], elem.color[1], elem.color[2]}}
    });
  }

  payload["draws"] = nlohmann::json::array();
  for (const auto & draw : frame.draws) {
    nlohmann::json item;
    switch (draw.type) {
      case UIDrawType::Point:
        item["type"] = "point";
        item["point"] = {draw.p1.x, draw.p1.y};
        item["radius"] = draw.radius;
        break;
      case UIDrawType::Points:
        item["type"] = "points";
        item["points"] = nlohmann::json::array();
        for (const auto & p : draw.points) {
          item["points"].push_back({p.x, p.y});
        }
        item["thickness"] = draw.thickness;
        break;
      case UIDrawType::Text:
        item["type"] = "text";
        item["text"] = draw.text;
        item["position"] = {draw.p1.x, draw.p1.y};
        item["font_scale"] = draw.font_scale;
        item["thickness"] = draw.thickness;
        break;
      case UIDrawType::Line:
        item["type"] = "line";
        item["p1"] = {draw.p1.x, draw.p1.y};
        item["p2"] = {draw.p2.x, draw.p2.y};
        item["thickness"] = draw.thickness;
        break;
    }
    item["color"] = {draw.color[0], draw.color[1], draw.color[2]};
    payload["draws"].push_back(std::move(item));
  }

  return payload;
}

void UIWebStream::sendFrame()
{
  if (!enabled_ || socket_ < 0) return;
  auto frame = UIStreamRecorder::instance().snapshot();
  auto payload = buildJson(frame);
  auto data = payload.dump();
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

}  // namespace tools
