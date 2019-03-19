#include "tracker.h"

Tracker::Tracker(cv::Mat imgL, cv::Mat imgR, cv::Rect roi)
{
  imgL.copyTo(_backgroundL);
  imgR.copyTo(_backgroundR);
  _roi = roi;

  cv::cvtColor(_backgroundL, _backgroundL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(_backgroundR, _backgroundR, cv::COLOR_BGR2GRAY);
}

void Tracker::setImages(cv::Mat imgL, cv::Mat imgR)
{
  imgL.copyTo(_imgL);
  imgR.copyTo(_imgR);
}

void Tracker::calcFinalPosition()
{
  cv::Point2f centerL{calcMoment(_imgL, _backgroundL)};
  cv::Point2f centerR{calcMoment(_imgR, _backgroundR)};
}

cv::Point2f Tracker::calcMoment(cv::Mat img, cv::Mat background)
{
  cv::Mat g_img;
  cv::cvtColor(img, g_img, cv::COLOR_BGR2GRAY);

  g_img = absoluteDifference(g_img, background);
  g_img = computeThreshold(g_img, 20);
  g_img = cleanUpNoise(g_img);

  cv::Moments m{cv::moments(g_img, true)};
  cv::Point2f center{m.m10/m.m00, m.m01/m.m00};

  return center;
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
