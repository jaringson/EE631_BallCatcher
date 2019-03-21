#include "tracker.h"

Tracker::Tracker(cv::Mat imgL, cv::Mat imgR, cv::Rect roiL, cv::Rect roiR)
{
  imgL.copyTo(_backgroundL);
  imgR.copyTo(_backgroundR);
  _roiL = roiL;
  _roiR = roiR;

  cv::cvtColor(_backgroundL, _backgroundL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(_backgroundR, _backgroundR, cv::COLOR_BGR2GRAY);
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
    // cv::undistortPoints(ptsL, outputL, camera_matL, dst_coeffL, R1, P1); //Need to get file for R,P,Q
    // cv::undistortPoints(ptsR, outputR, camera_matR, dst_coeffR, R2, P2);

    std::vector<cv::Point3f> pts = doPerspectiveTransform(outputL, outputR);
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
