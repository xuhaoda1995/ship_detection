//
// Created by jimiaozhou on 18-8-15.
//

#include "utils.h"

#define eps 1e-10

//Show image with opening an window
void show_image(const char *image_name, const cv::Mat &img, bool waitkey) {
	cv::namedWindow(image_name);
	cv::imshow(image_name, img);

	if (waitkey) {
		waitKey(0);
	}
}

void adaptiveMedianBlur(const Mat &src, Mat &dst, int kernelSize, int maxSize) {

}

void line_detection(const cv::Mat &src, std::vector<cv::Vec4i> &lines) {
	HoughLinesP(src, lines, 1, CV_PI / 180, 50, 30, 10);
}

//detect image egdes using canny or other method
void edge_detection(const cv::Mat &src, cv::Mat &dst, bool show) {
	double threshold1 = 180;
	double threshold2 = 255;
	cv::Canny(src, dst, threshold1, threshold2);

	if (show) {
		show_image("Gray Scale", dst);
	}

//	LineSegmentDetector
}

void drawBoundingBox(const Mat &src, const Rect &bounding_box, Mat &dst, bool show) {

}

bool intersection(const Vec4i &l, const Rect &rect) {
	Point lp_1(l[0], l[1]);
	Point lp_2(l[2], l[3]);

	Point rect_pts[4];

	rect_pts[0] =  Point(rect.x, rect.y); // left top
	rect_pts[1] = Point(rect.x + rect.width, rect.y); //right top
	rect_pts[2] = Point(rect.x, rect.y + rect.height); //left bottom
	rect_pts[3] = Point(rect.x + rect.width, rect.y + rect.height); //right bottom

	bool intersect = false;

	for(int i = 0;i<4;i++)
		if(inter(rect_pts[i],rect_pts[(i+1)%4],lp_1,lp_2)){//分别判断线段和矩形边是否相交
			intersect = true;
			break;
		}
	if(!intersect && (in(lp_1, rect_pts[0], rect_pts[3]) || in(lp_2, rect_pts[0], rect_pts[3])))//判断线段端点是否在矩形内部
		intersect = true;






	return intersect;
}

double multi(Point a, Point b, Point c) {
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

int inter(Point x, Point y, Point s, Point t) {//判断线段xy和线段st是否相交
	double a, b, c, d;
	if (min(x.x, y.x) > max(s.x, t.x) || min(x.y, y.y) > max(s.y, t.y) ||
		min(s.x, t.x) > max(x.x, y.x) || min(s.y, t.y) > max(x.y, y.y))
		return 0;
	a = multi(x, y, s);
	b = multi(x, y, t);
	c = multi(s, t, x);
	d = multi(s, t, y);
	return a * b < eps && c * d < eps;
}

int in(const Point &a, const Point &left_top, const Point &right_bottom) {//判断点a是否在矩形内部
	return a.x >= left_top.x && a.x <= right_bottom.x && a.y >= left_top.y && a.y <= right_bottom.y;
}
