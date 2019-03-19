#include "tracker.h"

Tracker::Tracker(cv::Mat imgL, cv::Mat imgR)
{
  imgL.copyTo(_backgroundL);
  imgR.copyTo(_backgroundR);
}

void Tracker::setImages(cv::Mat imgL, cv::Mat imgR)
{
  imgL.copyTo(_imgL);
  imgR.copyTo(_imgR);
}

void Tracker::calcFinalPosition()
{
  cv::Point2f centerL{calcMoment(_imgL)};
  cv::Point2f centerR{calcMoment(_imgR)};
}

cv::Point2f Tracker::calcMoment(const cv::Mat & img)
{

}
