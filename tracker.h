#ifndef TRACKER
#define TRACKER

#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

class Tracker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Tracker(cv::Mat imgL, cv::Mat imgR);
  void setImages(cv::Mat imgL, cv::Mat imgR);
  void calcFinalPosition();

private:
  cv::Point2f calcMoment(const cv::Mat& img);

  cv::Mat _backgroundL, _backgroundR;
  cv::Mat _imgL, _imgR;

};

#endif
