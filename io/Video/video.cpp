#include <libusb-1.0/libusb.h>

#include <video.hpp>

#include "tools/logger.hpp"
using namespace std::chrono_literals;
namespace io
{

Video::Video(std::string path) : quit_(false), ok_(false), queue_(1)
{
  this->path = path;
  video.open(this->path);
  open();
}
void Video::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}
void Video::open()
{
  (path);
  ok_ = true;
  capture_thread_ = std::thread{[this] {
      ok_ = true;
      while (!quit_) {
        int fps=video.get(cv::CAP_PROP_FPS);
        int c=1000/fps;
        std::chrono::milliseconds sleep_time(c);
      std::this_thread::sleep_for(sleep_time);

      cv::Mat img;

      auto timestamp = std::chrono::steady_clock::now();

      if (!video.isOpened()) {
        tools::logger()->warn("video dropped!");
        ok_ = false;
        break;
      }

      video>>img;

      queue_.push({img, timestamp});
    }
  }};

  tools::logger()->info("video opened.");
}
};  // namespace io
