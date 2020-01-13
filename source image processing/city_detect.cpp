#include "stdafx.h"
#include <iostream>
#include <opencv/cv.hpp>
#include <vector>

using namespace std;
using namespace cv;
cv::Scalar lower_range = { 0,  0, 0 };
cv::Scalar upper_range = { 10,  10, 10 };

string IntToStr(int data)
{
	char ch_str[10] = { 0 };
	string  str_data(ch_str);
	return str_data;
}


//初始化图形并圈出
string detect(vector<Point> cnts_single, vector<Point>& approx)
{
	string shape = "undentified";
	double peri = arcLength(cnts_single, true);
	approxPolyDP(cnts_single, approx, 0.015 * peri, true);
	if (approx.size() == 3)
	{
		shape = "triangle";
	}
	else if (approx.size() == 4)
	{
		shape = "rectangle";
		/*待完成*/
	}
	else if (approx.size() == 5)
	{
		shape = "pentagon";
	}
	else if (approx.size() > 10)
	{
		shape = "SHizi";
	}

	return shape;
}

int main()
{
	cv::Mat src = cv::imread("C:\\Users\\10027\\Pictures\\dst.jpg");
	cv::inRange(src, lower_range, upper_range, src);
	
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
	cv::dilate(src, src, element);

	Mat cimg;
	medianBlur(src, cimg, 5);
	GaussianBlur(cimg, cimg, Size(9, 9), 2, 2);
	//   medianBlur(cimg, cimg, 5);
	Canny(cimg, cimg, 10, 250, 5);
	
	vector<vector<Point>>cnts;//获取了一堆又一堆点
	findContours(cimg, cnts, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	for (int i = 0; i < cnts.size(); i++)
	{
		vector<Point> cnts_single = cnts[i];//获取了上面一堆点中的一个点
		if (cnts_single.size() > 0)
		{
			vector<Point> approx;
			string shape = detect(cnts_single, approx);
			Moments M = moments(cnts_single);
			int cX, cY;
			if (M.m10 != 0)
			{
				//表示图像重心
				cX = int((M.m10 / M.m00));
				cY = int((M.m01 / M.m00));
				cout << cX << ", " << cY << endl;
			}
			else
			{
				cX = cY = 0;
			}
			putText(cimg, IntToStr(cX) + " , " + IntToStr(cY), Point(cX, cY), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);  //质心位置

		}
		else
		{
			//printf("too fucking small %d\n", cnts_single.size());
		}
	}

	cv::imwrite("dst_1.jpg", src);
	std::system("pause");
	return 0;
}