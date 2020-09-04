#include "cvInclude.h"

void image_watch_demo()
{
	cv::Mat input = cv::imread("G:\\learn_opencv\\images\\girl.jpg", 1);  // -1：原图 0：灰度 1：BGR 2：深度 4：彩色 
	cv::cvtColor(input, input, cv::COLOR_BGR2GRAY);
	cv::Canny(input, input, 80, 100);
	cv::namedWindow("yan",cv::WINDOW_AUTOSIZE);
	cv::imshow("yan", input);
	cv::imwrite("G:\\learn_opencv\\images\\yan.jpg", input);
	cv::waitKey(0);
}