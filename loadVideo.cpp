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

  return 0;
}
