#include "tracker.h"

#include <string>

Tracker::Tracker(cv::Mat imgL, cv::Mat imgR, cv::Rect roiL, cv::Rect roiR)
{
  imgL.copyTo(_backgroundL);
  imgR.copyTo(_backgroundR);
  _roiL = roiL;
  _roiR = roiR;

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

void Tracker::calcCatcherPosition()
{

}

void Tracker::calcBallPosition()
{
  cv::Point2f centerL{calcMoment(_imgL, _backgroundL)};
  cv::Point2f centerR{calcMoment(_imgR, _backgroundR)};

  //reset ROI's in else statement
  if(centerL.x == 0 && centerL.y == 0 && centerR.x == 0 && centerR.y == 0) //assumes ball has a moment at 0, 0
  {
    centerL.x += _roiL.x;
    centerL.y += _roiL.y;
    centerR.x += _roiR.x;
    centerR.y += _roiR.y;

    std::vector<cv::Point2f> outputL, outputR;
    std::vector<cv::Point2f> ptsL{centerL};
    std::vector<cv::Point2f> ptsR{centerR};
    cv::undistortPoints(ptsL, outputL, _camera_matL, _dst_coeffL, _R1, _P1);
    cv::undistortPoints(ptsR, outputR, _camera_matR, _dst_coeffR, _R2, _P2);

    std::vector<cv::Point3f> pts = doPerspectiveTransform(outputL, outputR);

    pts[0].x -= 10.135; //Put into the center of the catchers frame. Need to flip x still
    pts[0].y -= 29.0;
    pts[0].z -= 21.0;

    //Add pts to the A matrix
  }
}

cv::Point2f Tracker::calcMoment(cv::Mat img, cv::Mat background)
{
  cv::Mat g_img;

  g_img = absoluteDifference(g_img, background);
  g_img = computeThreshold(g_img, 20);
  g_img = cleanUpNoise(g_img);

  cv::Moments m{cv::moments(g_img, true)};
  cv::Point2f center{m.m10/m.m00, m.m01/m.m00};

  return center;
}

std::vector<cv::Point3f> Tracker::doPerspectiveTransform(std::vector<cv::Point2f> ptsL, std::vector<cv::Point2f> ptsR)
{
  std::vector<cv::Point3f> final;

  std::vector<cv::Point3f> perspL;
  perspL.push_back(cv::Point3f(ptsL[0].x, ptsL[0].y, ptsL[0].x - ptsR[0].x));

  //This spits out the coordinates in the left camera frame
  cv::perspectiveTransform(perspL, final, _Q); //Get Q

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
