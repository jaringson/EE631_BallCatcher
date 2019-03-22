#include "tracker.h"
#include <string>

#define DEBUG

Tracker::Tracker(cv::Mat imgL, cv::Mat imgR)
{
  //could hardcode the roi's instead of passing in
  imgL.copyTo(_backgroundL);
  imgR.copyTo(_backgroundR);
  resetROI();

  cv::cvtColor(_backgroundL, _backgroundL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(_backgroundR, _backgroundR, cv::COLOR_BGR2GRAY);

  std::string filename{"../stereo_params.yaml"};
  cv::FileStorage fin(filename, cv::FileStorage::READ);
  fin["Camera_MatrixL"] >> _camera_matL;
  fin["Distortion_ParamsL"] >> _dst_coeffL;
  fin["Camera_MatrixR"] >> _camera_matR;
  fin["Distortion_ParamsR"] >> _dst_coeffL;
  fin["R"] >> _R;
  fin["T"] >> _T;
  fin["E"] >> _E;
  fin["F"] >> _F;
  fin["R1"] >> _R1;
  fin["R2"] >> _R2;
  fin["P1"] >> _P1;
  fin["P2"] >> _P2;
  fin["Q"] >> _Q;
}

void Tracker::setImages(cv::Mat imgL, cv::Mat imgR)
{
  cv::cvtColor(imgL, _imgL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(imgR, _imgR, cv::COLOR_BGR2GRAY);
}

cv::Point2f Tracker::calcCatcherPosition()
{
  Eigen::VectorXd bx = _pts.col(0);
  Eigen::VectorXd by = _pts.col(1);

  Eigen::Matrix<double, Eigen::Dynamic, 3> A;
  A.col(0) = _pts.col(2);
  A.col(1) = _pts.col(2);
  A.col(2) = Eigen::VectorXd::Constant(_pts.rows(), 1);

  Eigen::Vector3d x, y;
  //I think we want to solve with colPivHouseholderQR (more accurate) or HouseholderQR (faster)
  x = A.colPivHouseholderQr().solve(bx);
  y = A.colPivHouseholderQr().solve(by);

  // Eigen::RowVector3d pt; //Don't need to do matrix multiply. Just need last value in x and y
  // pt << 0.0, 0.0, 1;
  cv::Point2f pt{-x(2), y(2)}; //Negative sign to put in catcher frame
  return pt;
}

void Tracker::calcBallPosition()
{
  cv::Point2f centerL{calcMoment(_imgL(_roiL), _backgroundL(_roiL))};
  cv::Point2f centerR{calcMoment(_imgR(_roiR), _backgroundR(_roiR))};

  if(centerL.x == 0 && centerL.y == 0 && centerR.x == 0 && centerR.y == 0)
  {
      resetROI();
  }
  else
  {
    centerL.x += _roiL.x;
    centerL.y += _roiL.y;
    centerR.x += _roiR.x;
    centerR.y += _roiR.y;

#ifdef DEBUG
  cv::Mat cropL, cropR;
  _imgL(_roiL).copyTo(cropL);
  _imgR(_roiR).copyTo(cropR);
  cv::cvtColor(cropL, cropL, cv::COLOR_GRAY2BGR);
  cv::cvtColor(cropR, cropR, cv::COLOR_GRAY2BGR);


  cv::imshow("Left Crop", cropL);
  cv::imshow("Right Crop", cropR);
  cv::waitKey(0);
#endif

    std::vector<cv::Point2f> outputL, outputR;
    std::vector<cv::Point2f> ptsL{centerL};
    std::vector<cv::Point2f> ptsR{centerR};
    cv::undistortPoints(ptsL, outputL, _camera_matL, _dst_coeffL, _R1, _P1);
    cv::undistortPoints(ptsR, outputR, _camera_matR, _dst_coeffR, _R2, _P2);

    std::vector<cv::Point3f> pts = doPerspectiveTransform(outputL, outputR);

    pts[0].x -= 10.135; //Put into the center of the catchers frame. Maybe measure again
    pts[0].y -= 29.0;
    pts[0].z -= 21.0;

    //Add pts to the A matrix
    _pts.conservativeResize(_pts.rows() + 1, _pts.cols());
    _pts.row(_pts.rows() - 1) = Eigen::RowVector3d(pts[0].x, pts[0].y, pts[0].z);

    // TODO test roi update
    _roiL.x = (int(centerL.x) - 25 < 0) ? int(centerL.x) - 25 : 0;
    _roiL.y = (int(centerL.y) - 20 < 0) ? int(centerL.y) - 20 : 0;
    _roiL.width += 1;
    _roiL.height += 2;
    _roiR.x = (int(centerR.x - _roiR.width/2.0) < 0) ? int(centerR.x - _roiR.width/2.0) : 0;
    _roiR.y = (int(centerR.y) - 20 < 0) ? int(centerR.y) - 20 : 0;
    _roiR.width += 1;
    _roiR.height += 2;
  }
}

cv::Point2f Tracker::calcMoment(cv::Mat g_img, cv::Mat background)
{
  g_img = absoluteDifference(g_img, background);
  g_img = computeThreshold(g_img, 20);
  g_img = cleanUpNoise(g_img);

  cv::Moments m{cv::moments(g_img, true)};
  cv::Point2f center;
  if (m.m00 != 0.0)
    center = cv::Point2f{m.m10/m.m00, m.m01/m.m00};
  else
    center = cv::Point2f{0,0};

  return center;
}

std::vector<cv::Point3f> Tracker::doPerspectiveTransform(std::vector<cv::Point2f> ptsL, std::vector<cv::Point2f> ptsR)
{
  std::vector<cv::Point3f> final;

  std::vector<cv::Point3f> perspL;
  perspL.push_back(cv::Point3f(ptsL[0].x, ptsL[0].y, ptsL[0].x - ptsR[0].x));

  //This spits out the coordinates in the left camera frame
  cv::perspectiveTransform(perspL, final, _Q);

  return final;
}

cv::Mat Tracker::absoluteDifference(cv::Mat gray_frame, cv::Mat prev_frame)
{
  cv::Mat image;
  cv::absdiff(prev_frame, gray_frame, image);

  return image;
}

cv::Mat Tracker::computeThreshold(cv::Mat gray_frame, int thresh)
{
  int high_val{255}, type{0};
  cv::Mat image;
  cv::threshold(gray_frame, image, thresh, high_val, type);

  return image;
}

cv::Mat Tracker::cleanUpNoise(cv::Mat noisy_img)
{
  cv::Mat img;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::erode(noisy_img, img, element);
  cv::dilate(img, img, element);

  return img;
}

void Tracker::resetROI()
{
  _roiL.x = roiL_init_x;
  _roiL.y = roiL_init_y;
  _roiL.width = roi_init_w;
  _roiL.height = roi_init_h;

  _roiR.x = roiR_init_x;
  _roiR.y = roiR_init_y;
  _roiR.width = roi_init_w;
  _roiR.height = roi_init_h;
}
