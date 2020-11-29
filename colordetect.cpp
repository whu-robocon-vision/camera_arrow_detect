#include "colordetect.h"

colordetect::colordetect()
{

}

std::vector<cv::Point> colordetect::detect_arrow(cv::Mat &src)
{
    std::vector<cv::Point> points;
    cv::Mat hsv_frame;
    cv::cvtColor(src, hsv_frame, cv::COLOR_BGR2HSV);
    cv::Mat hsv_mask;
    cv::inRange(hsv_frame, cv::Scalar(0, 0, 0), cv::Scalar(180, 190, 70), hsv_mask);
    cv::Mat e33 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat morph_frame;
    // cv::morphologyEx(hsv_mask, morph_frame, cv::MORPH_OPEN, e33);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(hsv_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        cv::RotatedRect rect =  cv::minAreaRect(contours[i]);
        cv::Point2f *rect_points = new cv::Point2f[4];
        rect.points(rect_points);
        std::vector<cv::Point> box;
        for (size_t i = 0; i < 4; i++) {
            box.push_back(rect_points[i]);
        }
        float w = std::max(rect.size.width, rect.size.height);
        float h = std::min(rect.size.width, rect.size.height);
        int dir = 0;
        if (area >= 200 && area <= 6000 && w > 5 * h) {
            if (check_head(box, src, dir)) {
                cv::Moments moment = cv::moments(contours[i], false);
                cv::Point2f cp;
                cp.x = moment.m10 / moment.m00;
                cp.y = moment.m01 / moment.m00;
                cv::circle(src, cp, 3, cv::Scalar(0, 0, 255), -1);
                for (size_t i = 0; i < 4; i++) {
                    cv::line(src, rect_points[i], rect_points[(i + 1) % 4], cv::Scalar(255, 0, 0), 3);
                }
                points.push_back(cv::Point(cp));
            }
        }
    }
    return points;
}

bool colordetect::check_head(std::vector<cv::Point> &points, cv::Mat &frame, int &dir)
{
    std::vector<std::vector<cv::Point>> box = get_extend_box(points);
    std::vector<cv::Point> box1 = box[0];
    std::vector<cv::Point> box2 = box[1];
    bool flag1 = check_arrow_head(frame, box1);
    bool flag2 = check_arrow_head(frame, box2);
    // cv::drawContours(frame, box, 0, cv::Scalar(255, 255, 0), 3);
    // cv::drawContours(frame, box, 1, cv::Scalar(255, 255, 0), 3);
    if (flag1 && !flag2) {
        cv::drawContours(frame, box, 0, cv::Scalar(255, 255, 0), 3);
        dir = 0;
        return true;
    } else if (!flag1 && flag2) {
        cv::drawContours(frame, box, 1, cv::Scalar(255, 255, 0), 3);
        dir = 1;
        return true;
    } else {
        return false;
    }
}

std::vector<std::vector<cv::Point>> colordetect::get_extend_box(std::vector<cv::Point> box)
{
    std::vector<std::vector<cv::Point>> boxs;
    double d1 = get_distance_square(box[0], box[1]);
    double d2 = get_distance_square(box[1], box[2]);
    std::vector<cv::Point> box1, box2;
    box1.clear();
    box2.clear();
    boxs.clear();
    cv::Point p1, p2;
    if (d1 < d2) {
        p1 = get_extend_point(box[1], box[2], d1, d2);
        p2 = get_extend_point(box[0], box[3], d1, d2);
        box1.push_back(box[0]);
        box1.push_back(box[1]);
        box1.push_back(p1);
        box1.push_back(p2);
        p1 = get_extend_point(box[2], box[1], d1, d2);
        p2 = get_extend_point(box[3], box[0], d1, d2);
        box2.push_back(box[2]);
        box2.push_back(p1);
        box2.push_back(p2);
        box2.push_back(box[3]);
        boxs.push_back(box1);
        boxs.push_back(box2);
        return boxs;
    } else {
        p1 = get_extend_point(box[1], box[0], d2, d1);
        p2 = get_extend_point(box[2], box[3], d2, d1);
        box1.push_back(box[1]);
        box1.push_back(p1);
        box1.push_back(p2);
        box1.push_back(box[2]);
        p1 = get_extend_point(box[0], box[1], d2, d1);
        p2 = get_extend_point(box[3], box[2], d2, d1);
        box2.push_back(box[0]);
        box2.push_back(box[3]);
        box2.push_back(p2);
        box2.push_back(p1);
        boxs.push_back(box1);
        boxs.push_back(box2);
        return boxs;
    }
}

bool colordetect::check_arrow_head(cv::Mat &frame, std::vector<cv::Point> box)
{
    cv::Mat mask = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
    cv::Point points[4];
    for (size_t i = 0; i < 4; i++) points[i] = box[i];
    // int npoints[1] = {4};

    // cv::fillPoly(mask, (const cv::Point **)points, npoints, 0, cv::Scalar(255));
    cv::fillConvexPoly(mask, points, 4, cv::Scalar(255));
    // cv::imshow("mask", mask);
    cv::Scalar mean = cv::mean(frame, mask);
    if (mean[0] > 110 && mean[1] > 110 && mean[2] > 110)
        return true;
    return false;
}

double colordetect::get_distance_square(cv::Point p1, cv::Point p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

cv::Point colordetect::get_extend_point(cv::Point p1, cv::Point p2, double d1, double d2, double scale)
{
    cv::Point p;
    p.x = (int)((scale * d1 / d2) * (p1.x - p2.x) + p1.x);
    p.y = (int)((scale * d1 / d2) * (p1.y - p2.y) + p1.y);
    return p;
}
