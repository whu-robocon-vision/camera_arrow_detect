#ifndef COLORDETECT_H
#define COLORDETECT_H
#include "common.h"

class colordetect
{
public:
    colordetect();
    std::vector<cv::Point> detect_arrow(cv::Mat &src);
private:
    bool check_head(std::vector<cv::Point> &points, cv::Mat &frame, int &dir);
    std::vector<std::vector<cv::Point>> get_extend_box(std::vector<cv::Point> box);
    bool check_arrow_head(cv::Mat &frame, std::vector<cv::Point> box);
    double get_distance_square(cv::Point p1, cv::Point p2);
    cv::Point get_extend_point(cv::Point p1, cv::Point p2, double d1, double d2, double scale = 15);
};

#endif // COLORDETECT_H
