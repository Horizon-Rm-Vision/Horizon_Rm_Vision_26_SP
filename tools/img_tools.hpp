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

class Recode_video
{
private:
  double fps;
  std::string outputPath;
  int frameWidth;
  int frameHeight;  
  int fourcc;
  bool frist=true;
  cv::VideoWriter writer;

public:
  Recode_video(double fps,std::string path) { this->fps = fps;this->outputPath=path; }
  ~Recode_video() { writer.release(); }
  void Recode_open(cv::Mat firstimg)
  {
    cv::Size frameSize = firstimg.size();
    frameWidth = frameSize.width;
    frameHeight = frameSize.height; 
    fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    
    this->writer.open(outputPath, fourcc, fps, frameSize, true);
    if (!writer.isOpened()) {
     
      std::cerr << "Could not open the output video file for write\n";
    }
    frist=false;
  }
  void Recode_Fin(cv::Mat img)
  {
        if(frist)
        {
            Recode_open(img);
           
        }
        else
        {
            Recode_in(img);
        }
        
  }
  void Recode_in(cv::Mat img)
  {
   
    if (!writer.isOpened()) {
      std::cerr << "Video writer is not opened. Please call Recode_open first.\n";
      return;
    }
    writer.write(img);
  }
  void Recode_close() { writer.release(); }
};

// class Recode_TxT
// {
//   private:
//   std::string name;
//   public:
//   void Recode()
//   {

//   }

// }
}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP