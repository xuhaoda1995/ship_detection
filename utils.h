//
// Created by jimiaozhou on 18-8-15.
//

#ifndef SHIPDETECTION_UTILS_H
#define SHIPDETECTION_UTILS_H

//#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

void show_image(const char* image_name, const cv::Mat& img, bool waitkey= true);

void adaptiveMedianBlur(const Mat &src, Mat& dst, int kernelSize,int maxSize);

void line_detection(const cv::Mat& src, std::vector<cv::Vec4i>& lines);

void edge_detection(const cv::Mat& src, cv::Mat& dst, bool show= false);

void drawBoundingBox(const Mat& src, const Rect& bounding_box, Mat& dst, bool show = false);

bool intersection(const Vec4i& l, const Rect& rect);

double multi(Point a, Point b, Point c);
int inter(Point x, Point y, Point s, Point t);
int in(const Point &a, const Point &left_top, const Point &right_bottom);


#endif //SHIPDETECTION_UTILS_H
