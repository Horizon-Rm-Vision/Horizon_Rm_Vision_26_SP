#ifndef IO__VIDEO_HPP
#define IO__VIDEO_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Video : public CameraBase
{
public:
      Video(std::string path);
//    ~Video() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
 
  private:
  cv::VideoCapture video;
std::thread capture_thread_;
  std::thread daemon_thread_;
  struct CameraData
  {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
    };
    std::string path;
    bool quit_, ok_;
    tools::ThreadSafeQueue<CameraData> queue_;
    void open();

};

}  // namespace io

#endif  // IO__VIDEO_HPP