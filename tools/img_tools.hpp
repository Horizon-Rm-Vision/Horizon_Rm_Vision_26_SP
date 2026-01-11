#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tools
{
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

class draw_picture
   {
      private:
      cv::Point2f position;
      cv::Mat frame;
      cv::Scalar highlight_color = cv::Scalar(0, 165, 255); // 橙色
      cv::Scalar text_color = cv::Scalar(0, 255, 0); // 绿色
      public:
      void draw_set_img(cv::Mat img)
      {
        this->frame=img;
      }
      void draw_TxT(std::string S,std::string data,bool im=false);
      void draw_TxT(std::string S,bool im=false);
      void draw_S(std::string S,float data,bool im=false);
      void draw_S(std::string S1,float data1,std::string S2,float data2,bool im=false);
      void draw_ChangeX(float positionx=50);
      void draw_Picture_center(cv::Mat frame,cv::Point2f point,cv::Scalar  color_center= cv::Scalar(0, 0, 255))
    {
           cv::circle(frame,point,8, color_center, -1); 
    }
      void draw_clean()
      {
        position.x=0;
        position.y=30;
      }
      draw_picture()
      {
          position.x=0;
          position.y=30;
      }
};
}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP