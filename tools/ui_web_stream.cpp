#include "ui_web_stream.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "yaml.hpp"

namespace tools {

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

void UIWebStream::initSocket()
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port_);
  destination_.sin_addr.s_addr = ::inet_addr(host_.c_str());

  UIStreamRecorder::instance().setEnabled(true);
  UIManager::setGlobalCaptureEnabled(true);
}

void UIWebStream::closeSocket()
{
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
