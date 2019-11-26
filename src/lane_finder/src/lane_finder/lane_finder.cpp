#include "lane_finder.h"
#include "PolyfitBoost.hpp"
#include "PolyfitEigen.hpp"

int i = 0;

LaneFinder::LaneFinder(/* args */)
{
}

LaneFinder::~LaneFinder()
{
}

float LaneFinder::findLanes(cv_bridge::CvImagePtr cv_ptr)
{
    
    cv::Point2f srcRect[4];
    srcRect[0] = cv::Point2f((cv_ptr->image.cols) * 0.30f, (cv_ptr->image.rows) / 2);
    srcRect[1] = cv::Point2f((cv_ptr->image.cols) * 0.65f, (cv_ptr->image.rows) / 2);
    srcRect[2] = cv::Point2f(0.f, cv_ptr->image.rows - 20);
    srcRect[3] = cv::Point2f((cv_ptr->image.cols), cv_ptr->image.rows - 20);
    cv::line(cv_ptr->image, srcRect[0], srcRect[1], cv::Scalar(255, 0, 0));
    cv::line(cv_ptr->image, srcRect[1], srcRect[3], cv::Scalar(255, 0, 0));
    cv::line(cv_ptr->image, srcRect[3], srcRect[2], cv::Scalar(0, 0, 255));
    cv::line(cv_ptr->image, srcRect[2], srcRect[0], cv::Scalar(255, 0, 0));

    cv::Point2f dstRect[4];
    dstRect[0] = cv::Point2f(0.f, 0.f);
    dstRect[1] = cv::Point2f((cv_ptr->image.cols), 0.f);
    dstRect[2] = cv::Point2f(0, (cv_ptr->image.rows));
    dstRect[3] = cv::Point2f((cv_ptr->image.cols), (cv_ptr->image.rows));

    cv::Mat warp_mat = cv::getPerspectiveTransform(srcRect, dstRect);
    cv::Mat warp_dst = cv::Mat::zeros((cv_ptr->image.rows), (cv_ptr->image.cols), cv_ptr->image.type());
    cv::warpPerspective(cv_ptr->image, warp_dst, warp_mat, warp_dst.size());

    cv::cvtColor(warp_dst, warp_dst, cv::COLOR_BGR2GRAY);
    cv::threshold(warp_dst, warp_dst, 125, 255, cv::THRESH_BINARY);
    // cv::adaptiveThreshold(warp_dst, warp_dst, 125, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 12);
    // std::vector<std::vector<cv::Point> > contours;
    // std::vector<cv::Vec4i> hierarchy;
    // cv::RNG rng(12345);
    // cv::findContours(warp_dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // /// Draw contours
    // cv::Mat drawing = cv::Mat::zeros(warp_dst.size(), CV_8UC3);
    // for (int i = 0; i < contours.size(); i++)
    // {
    //     cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    //     drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
    // }

    int rect_height = warp_dst.rows / 6;
    int rect_width = warp_dst.rows / 6;

    int left_rect = 0;
    int left_count = 0;
    int left_x_count = 0;
    int left_x = 0;
    int right_rect = 0;
    int right_count = 0;
    int right_x = 0;
    int right_x_count = 0;
    std::vector<cv::Rect> left_rects;
    std::vector<cv::Rect> right_rects;
    std::vector<double> left_rects_x;
    std::vector<double> left_rects_y;
    std::vector<double> right_rects_x;
    std::vector<double> right_rects_y;

    cv::Mat colored = cv::Mat::zeros((cv_ptr->image.rows), (cv_ptr->image.cols), cv_ptr->image.type());
    cv::cvtColor(warp_dst, colored, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < warp_dst.rows; i++)
    {
        if ((i + 1) % rect_height == 0)
        {
            if (left_x_count > 0 && right_x_count > 0)
            {
                cv::Rect left_rect_cv(left_x / left_x_count - rect_width / 2, i - rect_height + 1, rect_width,
                                      rect_height);
                cv::Rect right_rect_cv(right_x / right_x_count - rect_width / 2, i - rect_height + 1, rect_width,
                                       rect_height);
                cv::rectangle(colored, left_rect_cv, cv::Scalar(0, 0, 255));
                cv::rectangle(colored, right_rect_cv, cv::Scalar(0, 0, 255));
                left_rects.push_back(left_rect_cv);
                right_rects.push_back(right_rect_cv);
                left_rects_x.push_back(left_rect_cv.x - warp_dst.rows/2);
                left_rects_y.push_back(left_rect_cv.y);
                right_rects_x.push_back(right_rect_cv.x - warp_dst.rows/2);
                right_rects_y.push_back(right_rect_cv.y);
            }
            left_x = 0;
            left_x_count = 0;
            left_rect = 0;
            left_count = 0;
            right_x = 0;
            right_x_count = 0;
            right_rect = 0;
            right_count = 0;
        }
        for (int j = 0; j < warp_dst.cols; j++)
        {
            if (warp_dst.at<bool>(i, j))
            {
                if (j < warp_dst.cols / 2)
                {
                    left_rect += j;
                    left_count++;
                }
                if (j >= warp_dst.cols / 2)
                {
                    right_rect += j;
                    right_count++;
                }
            }
        }
        if (left_count > 0)
        {
            left_x += left_rect / left_count;
            left_x_count++;
            left_rect = 0;
            left_count = 0;
        }
        if (right_count > 0)
        {
            right_x += right_rect / right_count;
            right_x_count++;
            right_rect = 0;
            right_count = 0;
        }
    }
    std::reverse(left_rects_x.begin(),left_rects_x.end());
    std::reverse(right_rects_x.begin(),right_rects_x.end());
    std::vector<double> left_coeff = polyfit_boost(left_rects_y, left_rects_x, 3);
    std::vector<double> right_coeff = polyfit_boost(right_rects_y, right_rects_x, 3);

    for (int i = 0; i < left_rects.size(); i++)
    {
        std::cout<<"\nleft rect x: "<< left_rects.at(i).x;
        std::cout<<"\nleft rect y: "<< left_rects.at(i).y;
    }
    for (int i = 0; i < right_rects.size(); i++)
    {
        std::cout<<"\nright rect x: "<< right_rects.at(i).x;
        std::cout<<"\nright rect y: "<< right_rects.at(i).y;
    }
    
    std::cout << "\nCoefficients:\n";
    for (auto it = left_coeff.begin(); it != left_coeff.end(); ++it)
        std::cout << *it << " ";
    std::cout << "\nCoefficients weighted:\n";
    for (auto it = right_coeff.begin(); it != right_coeff.end(); ++it)
        std::cout << *it << " ";
    cv::imshow("Warped", colored);
    cv::imshow("Image", cv_ptr->image);
    cv::waitKey(3);
    return 0;
}
