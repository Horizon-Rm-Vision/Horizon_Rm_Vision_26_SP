#pragma once

#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

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

private:
  void initSocket();
  void closeSocket();
  nlohmann::json buildJson(const UIStreamFrame & frame) const;

  bool enabled_{false};
  int socket_{-1};
  sockaddr_in destination_{};
  std::string host_{"127.0.0.1"};
  uint16_t port_{9876};

  std::vector<UIElement> left_elements_;
  std::vector<UIElement> right_elements_;
  std::string program_mode_{"default"};
  int left_y_offset_{30};
  int right_y_offset_{30};
  int line_height_{25};
  double font_scale_{0.6};
  int thickness_{2};
};

}  // namespace tools
