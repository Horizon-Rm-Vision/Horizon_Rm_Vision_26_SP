#ifndef TOOLS__UI_HPP
#define TOOLS__UI_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <vector>
#include <optional>
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/target.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace tools
{
class Drawui
{
public:
    Drawui(
        cv::Mat & img, 
        const std::vector<auto_aim::Target> & targets, 
        const std::vector<auto_aim::Armor> & armors,
        const io::GimbalState & gs,
        const auto_aim::Plan & plan,
        float fps);
    ~Drawui();

    void drawText();
private:
    cv::Mat & img;
    const std::vector<auto_aim::Target> & targets;
    const std::vector<auto_aim::Armor> & armors;
    const io::GimbalState & gs;
    const auto_aim::Plan & plan;
    float fps;

    int y_offset = 30;
    int line_height = 25;
    cv::Scalar text_color{0, 255, 0}; // 绿色文本
    cv::Scalar highlight_color{0, 165, 255}; // 橙色高亮文本
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;
};
}  // namespace tools

#endif  // TOOLS__UI_HPP