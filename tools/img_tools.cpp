#include "img_tools.hpp"

namespace tools
{
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  cv::circle(img, point, radius, color, -1);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  std::vector<cv::Point> int_points(points.begin(), points.end());
  draw_points(img, int_points, color, thickness);
}

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

void draw_picture::draw_TxT(std::string S, std::string data,bool im)
{
   std::stringstream s;
   s << S << ":" << data;

   if (im==true)
    cv::putText(frame, s.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->highlight_color, 2);
   else
    cv::putText(frame, s.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->text_color, 2);
   position.y += 25;
}
void draw_picture::draw_TxT(std::string S,bool im)
{
   std::stringstream s;
   s << S ;

   if (im==true)
    cv::putText(frame, s.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6,this->highlight_color, 2);
   else
    cv::putText(frame, s.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->text_color, 2);
   position.y += 25;
}
void draw_picture::draw_S(std::string S, float data,bool im)
{
   std::stringstream s;
   s << S << ":" << data;
   if (im==true)
     cv::putText(frame, s.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->highlight_color, 2);
   else
      cv::putText(frame, s.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->text_color, 2);

   position.y += 25;
}
void draw_picture::draw_S(std::string S1,float data1,std::string S2,float data2,bool im)
{
   std::stringstream s1,s2;
   s1 << S1 << ":" << data1;
   s2 << S2 << ":" << data2;
   cv::Point2f hanshow;
   hanshow.x=this->position.x+250;
   hanshow.y=this->position.y;
   if (im==true)
   {
     cv::putText(frame, s1.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->highlight_color, 2);
     cv::putText(frame, s2.str(),  hanshow , cv::FONT_HERSHEY_SIMPLEX, 0.6, this->highlight_color, 2);

   }
   else
   {
     cv::putText(frame, s1.str(), this->position, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->text_color, 2);
     cv::putText(frame, s2.str(), hanshow, cv::FONT_HERSHEY_SIMPLEX, 0.6, this->text_color, 2);
   }

   position.y += 25;
}

void draw_picture::draw_ChangeX(float positionx)
{
   position.x = positionx;
   position.y = 30;
}
}  // namespace tools