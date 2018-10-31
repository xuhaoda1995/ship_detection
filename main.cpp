//standard cpp headers
#include <iostream>
#include <vector>
#include <string>

//OpenCV headers
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "utils.h"
#include "MorphoFilter.h"

void image_binary(const cv::Mat &src, cv::Mat &dst);
void median_blur(const cv::Mat &src, cv::Mat &dst, int ks);

int main()
{
    // tracker();
    const char *img_path = "images/test1.jpg";
    cv::Mat origin_img = cv::imread(img_path, -1);

    cv::Mat gray_img;
    cv::cvtColor(origin_img, gray_img, cv::COLOR_BGR2GRAY);
    std::cout << "image size " << gray_img.size << std::endl;
    show_image("Gray Scale", gray_img, true);

    MorphoFilter morpho_filter;
    Mat filterd_img;
    edge_detection(gray_img, filterd_img,true);

    Mat dilate_res;
    morpho_filter.dilate(filterd_img, dilate_res, 16, MORPH_CROSS, true);
    morpho_filter.erode(dilate_res, filterd_img, 18, MORPH_CROSS, true);
    Mat binary_reslut;
    median_blur(filterd_img, binary_reslut, 9);

    Mat dilate_result;
    morpho_filter.dilate(binary_reslut, dilate_result, 30, 2, true);
    std::vector<std::vector<Point>> contours;
    findContours(dilate_result, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    std::cout << "Find " << contours.size() << " Contours" << std::endl;

    Mat contours_result;
    origin_img.copyTo(contours_result);
    drawContours(contours_result, contours, -1, cv::Scalar(0, 0, 255));
    show_image("Gray Scale", contours_result);

    Mat rect_result = origin_img.clone();

    vector<Rect> rects;
    vector<int> intersect_index(contours.size(), -1);

    int contour_index = 0;
    for (auto &contour : contours)
    {
        Rect rect = boundingRect(contour);
        rects.push_back(rect);

        //        for (int i = 0; i < lines.size(); ++i) {
        //            if(intersection(lines[i], rect))
        //                intersect_index[contour_index] = i;
        //        }
        if (rect.width / float(rect.height) < 1.5 || rect.width / float(rect.height) > 10)
            continue;
        contour_index++;
        std::cout << rect.x << "\t" << rect.y << "\t" << rect.width << "\t" << rect.height << std::endl;
        //        cv::rectangle(rect_result, rect, Scalar(0, 0, 255), 3, 8, 0);
        cv::rectangle(rect_result, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height),
                      Scalar(0, 0, 255), 3);

        std::string text = std::to_string(contour_index);
        cv::putText(rect_result, text.c_str(), Point(rect.x, rect.y + 20),
                    FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    }
    show_image("Bounding Box", rect_result);
    /*
    Mat binary_reslut;
    morpho_filter.binary(gray_img, binary_reslut);

    cv::Mat canny_result;
    edge_detection(binary_reslut, canny_result);
    show_image("Canny Result", canny_result);


    vector<Vec4i> lines;
    line_detection(canny_result, lines);
    std::cout << "Lines Count: " << lines.size() << std::endl;

    //find max and min y-value of lines
    int min_h = 10000, max_h = -1;
    int max_index = 0, min_index = 0;
    Mat line_map = origin_img.clone();
    for(int i = 0; i< lines.size(); ++i) {
        line(line_map, cv::Point(lines[i][0], lines[i][1]),
             cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255));
        if(lines[i][1] > max_h) {
            max_h = lines[i][1];
            max_index = i;
        }
        if(lines[i][3] > max_h)
        {
            max_h = lines[i][3];
            max_index = i;
        }

        if(lines[i][1] < min_h)
        {
            min_h = lines[i][1];
            min_index = i;
        }
        if(lines[i][3] < min_h)
        {
            min_h = lines[i][3];
            min_index = i;
        }
    }
    show_image("Hough Lines", line_map);

    cv::Mat roi;
    cv::Mat roi_rgb;
    gray_img(cv::Rect(0, min_h - 20, gray_img.cols, max_h - min_h + 15)).copyTo(roi);
    origin_img(cv::Rect(0, min_h - 20, gray_img.cols, max_h - min_h + 15)).copyTo(roi_rgb);

    show_image("ROI", roi);

    Mat filter_result_roi;
    morpho_filter.filter(roi, filter_result_roi, true);

    Mat binary_result_roi;
    morpho_filter.binary(filter_result_roi, binary_result_roi, true);
    for(auto& l: lines) {
        l[1] = l[1] - (min_h - 20);
        l[3] = l[3] - (min_h - 20);

        line(binary_result_roi, Point(l[0], l[1]), Point(l[2], l[3]), cv::Scalar(255), 3);
    }
    show_image("Lines", binary_result_roi);

    Mat dilate_result;
    morpho_filter.dilate(binary_result_roi, dilate_result, 3, 2, true);

//    Mat line_map_roi = Mat::zeros(dilate_result.rows, dilate_result.cols, CV_8UC1);


//    morpho_filter.dilate(line_map_roi, line_map_roi, 5, 2, false);
//    show_image("Lines", line_map_roi);

//    Mat img_add = line_map_roi + dilate_result;
//    for(int i = 0; i < img_add.rows; i++) {
//        for(int j = 0; j < img_add.cols; j++) {
//            if(img_add.at<uchar >(i, j) > 255) {
//                img_add.at<uchar >(i, j) = 255;
//            } else {
//                img_add.at<uchar >(i, j) = 0;
//            }
//        }
//    }
//
//    imshow("add result", img_add);


    std::vector<std::vector<Point>> contours;
    findContours(dilate_result, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    std::cout << "Find " << contours.size()  << " Contours" << std::endl;

    Mat contours_result;
    roi_rgb.copyTo(contours_result);
    drawContours(contours_result, contours, -1, cv::Scalar(0, 0, 255));
    show_image("Contours result", contours_result);

    Mat rect_result = roi_rgb.clone();



    vector<Rect> rects;
    vector<int> intersect_index(contours.size(), -1);

    int contour_index = 0;
    for(auto& contour: contours) {
        Rect rect = boundingRect(contour);
        rects.push_back(rect);

        for (int i = 0; i < lines.size(); ++i) {
            if(intersection(lines[i], rect))
                intersect_index[contour_index] = i;
        }
        if(rect.width / float(rect.height) < 2  || rect.width / float(rect.height) > 15)
            continue;
        contour_index++;
        std::cout << rect.x << "\t" << rect.y << "\t" << rect.width << "\t" << rect.height << std::endl;
//        cv::rectangle(rect_result, rect, Scalar(0, 0, 255), 3, 8, 0);
        cv::rectangle(rect_result, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height),
                Scalar(0, 0, 255), 3);

        std::string text = std::to_string(contour_index);
        cv::putText(rect_result, text.c_str(), Point(rect.x, rect.y + 20),
                FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    }

//    vector<vector<int>> intersect_indices;
//
//    for(auto& l : lines) {
//        vector<int > indices;
//        for(int i =0; i < rects.size(); i++) {
//            if(intersection(l, rects[i])) {
//                indices.push_back(i);
//            }
//        }
//        if (indices.empty()) {
//            indices.push_back(-1);
//        }
//        intersect_indices.push_back(indices);
//    }
//
//    vector<Rect> merged_result;
//    for(int i = 0; i < intersect_indices.size(); i++) {
//        vector<int> indices = intersect_indices[i];
//        if (indices.size() == 1 && indices[0] != -1) {
//            merged_result.push_back(rects[indices[0]]);
//            break;
//        }
//        for(int j = 0; j < indices.size(); j++) {
//            if(indices[j] == -1)
//                break;
//
//        }
//    }
    show_image("Bounding Box", rect_result);
*/

    return 0;
}

void median_blur(const cv::Mat &src, cv::Mat &dst, int ks)
{
    cv::medianBlur(src, dst, ks);
    show_image("Gray Scale", dst);
}
