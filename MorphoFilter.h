//
// Created by jimiaozhou on 18-8-15.
//

#ifndef SHIPDETECTION_MORPHOFILTER_H
#define SHIPDETECTION_MORPHOFILTER_H

#include <iostream>
#include <vector>

#include "utils.h"

using namespace std;

int tracker();

class MorphoFilter {
public:
	MorphoFilter();
	void filter(const Mat& src, Mat& dst, bool show= false);
	void dilate(const Mat& src, Mat& dst, int kernel_size, int morph_shape, bool show=false);
	void erode(const Mat& src, Mat& dst, int kernel_size, int morph_shape, bool show=false);

	void binary(const Mat& src, Mat& dst, bool show= false);
};


#endif //SHIPDETECTION_MORPHOFILTER_H
