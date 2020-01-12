#include "stdafx.h"
#include <iostream>
#include <opencv/cv.hpp>
using namespace std;
cv::Scalar lower_range = { 50,  50, 20 };
cv::Scalar upper_range = { 150,  90, 50 };
int main()
{
	cv::Mat src = cv::imread("C:\\Users\\10027\\Pictures\\map.png");
	cv::Mat out;
	cv::inRange(src, lower_range, upper_range, out);
	cv::imwrite("dst.jpg", out);
	system("pause");
	return 0;
}