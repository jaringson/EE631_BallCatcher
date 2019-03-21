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

  Tracker(cv::Mat imgL, cv::Mat imgR, cv::Rect roiL, cv::Rect roiR);
  void setImages(cv::Mat imgL, cv::Mat imgR);
  void calcCatcherPosition();
  void calcBallPosition();

private:
  cv::Point2f calcMoment(cv::Mat img, cv::Mat background);
  cv::Mat absoluteDifference(cv::Mat gray_frame, cv::Mat prev_frame);
  cv::Mat computeThreshold(cv::Mat gray_frame, int thresh);
  cv::Mat cleanUpNoise(cv::Mat noisy_img);
  std::vector<cv::Point3f> doPerspectiveTransform(std::vector<cv::Point2f> ptsL, std::vector<cv::Point2f> ptsR);

  cv::Mat _backgroundL, _backgroundR;
  cv::Mat _imgL, _imgR;
  cv::Rect _roiL, _roiR;
  cv::Mat _camera_matL, _camera_matR, _dst_coeffL, _dst_coeffR;
  cv::Mat _R, _T, _E, _F;
  cv::Mat _R1, _R2, _P1, _P2, _Q;

};

#endif
