#include "tracker.h"
#include <opencv2/opencv.hpp>
#include <string>

#define DEBUG

int main()
{
  std::string filenameL("../baseball/ballL"), filenameR("../baseball/ballR");
  std::string file_ext(".bmp");

  cv::Mat imgL, imgR, backgroundL, backgroundR;
  imgL = cv::imread(filenameL + "00" + file_ext);
  imgR = cv::imread(filenameR + "00" + file_ext);

  Tracker tracker(imgL, imgR);

  for(int i(0); i < 100; i++)
  {
    std::string file_num;
    if(i < 10)
      file_num = "0" + std::to_string(i);
    else
      file_num = std::to_string(i);

    imgL = cv::imread(filenameL + file_num + file_ext);
    imgR = cv::imread(filenameR + file_num + file_ext);
  }

  return 0;
}
