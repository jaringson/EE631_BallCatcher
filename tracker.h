#ifndef TRACKER
#define TRACKER

#include <cmath>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

class Tracker
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Tracker();
  Tracker(cv::Mat imgL, cv::Mat imgR);
  void setImages(cv::Mat imgL, cv::Mat imgR);
  cv::Point2f calcCatcherPosition();
  bool calcBallPosition();

private:
  cv::Point2f calcMoment(cv::Mat img, cv::Mat background);
  cv::Mat absoluteDifference(cv::Mat gray_frame, cv::Mat prev_frame);
  cv::Mat computeThreshold(cv::Mat gray_frame, int thresh);
  cv::Mat cleanUpNoise(cv::Mat noisy_img);
  std::vector<cv::Point3f> doPerspectiveTransform(std::vector<cv::Point2f> ptsL, std::vector<cv::Point2f> ptsR);
  void resetROI();

  int roiL_init_x{345}, roiL_init_y{85}, roiR_init_x{265}, roiR_init_y{85}, roi_init_w{50}, roi_init_h{40};
  cv::Mat _backgroundL, _backgroundR;
  cv::Mat _imgL, _imgR;
  cv::Rect _roiL, _roiR;
  cv::Mat _camera_matL, _camera_matR, _dst_coeffL, _dst_coeffR;
  cv::Mat _R;
  cv::Mat _T;
  cv::Mat E;
  cv::Mat F;
  cv::Mat _R1, _R2, _P1, _P2, _Q;
  Eigen::Matrix<double, Eigen::Dynamic, 3> _pts;
  int _counter;
  int _back_count;

};

#endif
