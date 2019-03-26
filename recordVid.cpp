#include <opencv2/opencv.hpp>
#include <string>

int main()
{
  cv::VideoCapture cap(1);

  std::string filename ("baseball_catcher.avi");
  int ex = cv::VideoWriter::fourcc('M', 'P', 'E', 'G');
  cv::Size size(cap.get(3), cap.get(4));
  cv::VideoWriter v_out(filename, ex, 30, size, true);

  cv::Mat frame;
  int key;
  while(true)
  {
    cap >> frame;
    cv::imshow("Frame", frame);
    key = cv::waitKey(30);
    if(key == (int)('q'))
      break;
    v_out << frame;
  }
  v_out.release();
}
