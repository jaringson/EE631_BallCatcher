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

  Tracker(cv::Mat imgL, cv::Mat imgR, cv::Rect roi);
  void setImages(cv::Mat imgL, cv::Mat imgR);
  void calcFinalPosition();

private:
  cv::Point2f calcMoment(cv::Mat img, cv::Mat background);
  cv::Mat absoluteDifference(cv::Mat gray_frame, cv::Mat prev_frame);
  cv::Mat computeThreshold(cv::Mat gray_frame, int thresh);
  cv::Mat cleanUpNoise(cv::Mat noisy_img);

  cv::Mat _backgroundL, _backgroundR;
  cv::Mat _imgL, _imgR;
  cv::Rect _roi;

};

#endif
