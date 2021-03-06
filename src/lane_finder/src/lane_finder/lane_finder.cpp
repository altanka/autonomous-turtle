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

autonomous_msgs::LaneInfo LaneFinder::findLanes(cv_bridge::CvImagePtr cv_ptr, autonomous_msgs::LaneInfo lanes, float k1,
                                                float k2)
{
    cv::Point2f srcRect[4];
    srcRect[0] = cv::Point2f((cv_ptr->image.cols) * k1 - 20, (cv_ptr->image.rows) * 0.55f);
    srcRect[1] = cv::Point2f((cv_ptr->image.cols) * k2 + 20, (cv_ptr->image.rows) * 0.55f);
    srcRect[2] = cv::Point2f(0, cv_ptr->image.rows - 40);
    srcRect[3] = cv::Point2f((cv_ptr->image.cols), cv_ptr->image.rows - 40);
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
    std::vector<double> left_lane_x;
    std::vector<double> left_lane_y;
    std::vector<double> right_lane_x;
    std::vector<double> right_lane_y;

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
                left_rects_x.push_back(left_x / left_x_count - warp_dst.cols / 2);
                left_rects_y.push_back(i - rect_height + 1);
                right_rects_x.push_back(right_x / right_x_count - warp_dst.cols / 2);
                right_rects_y.push_back(i - rect_height + 1);
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
                    left_lane_x.push_back(j - warp_dst.cols / 2);
                    left_lane_y.push_back(i);
                    colored.at<cv::Vec3b>(i, j)[0] = 255;
                    colored.at<cv::Vec3b>(i, j)[1] = 0;
                    colored.at<cv::Vec3b>(i, j)[2] = 0;
                }
                if (j >= warp_dst.cols / 2)
                {
                    right_rect += j;
                    right_count++;
                    right_lane_x.push_back(j - warp_dst.cols / 2);
                    right_lane_y.push_back(i);
                    colored.at<cv::Vec3b>(i, j)[0] = 0;
                    colored.at<cv::Vec3b>(i, j)[1] = 255;
                    colored.at<cv::Vec3b>(i, j)[2] = 0;
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

    std::vector<double> left_coeff;
    std::vector<double> right_coeff;

    if (left_lane_x.size() > 0 && left_lane_y.size() > 0 && right_lane_x.size() > 0 && right_lane_y.size() > 0)
    {
        std::reverse(left_lane_x.begin(), left_lane_x.end());
        std::reverse(left_lane_y.begin(), left_lane_y.end());
        std::reverse(right_lane_x.begin(), right_lane_x.end());
        std::reverse(right_lane_y.begin(), right_lane_y.end());
        left_coeff = polyfit_boost(left_lane_y, left_lane_x, 3);
        right_coeff = polyfit_boost(right_lane_y, right_lane_x, 3);
    }
    else
    {
        std::cout << "Left rects x size : " << left_rects_x.size() << std::endl;
        std::cout << "Left rects y size : " << left_rects_x.size() << std::endl;
        std::cout << "Right rects x size: " << left_rects_x.size() << std::endl;
        std::cout << "Right rects y size: " << left_rects_x.size() << std::endl;
        left_coeff.push_back(0);
        left_coeff.push_back(0);
        left_coeff.push_back(0);
        left_coeff.push_back(0);
        right_coeff.push_back(0);
        right_coeff.push_back(0);
        right_coeff.push_back(0);
        right_coeff.push_back(0);
    }
    // try
    // {
    //     std::reverse(left_rects_x.begin(), left_rects_x.end());
    //     std::reverse(right_rects_x.begin(), right_rects_x.end());
    //     left_coeff = polyfit_boost(left_rects_y, left_rects_x, 3);
    //     right_coeff = polyfit_boost(right_rects_y, right_rects_x, 3);
    // }
    // catch (const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    //     std::cout << "Left rects x size : " << left_rects_x.size() << std::endl;
    //     std::cout << "Left rects y size : " << left_rects_x.size() << std::endl;
    //     std::cout << "Right rects x size: " << left_rects_x.size() << std::endl;
    //     std::cout << "Right rects y size: " << left_rects_x.size() << std::endl;
    //     left_coeff.push_back(0);
    //     left_coeff.push_back(0);
    //     left_coeff.push_back(0);
    //     left_coeff.push_back(0);
    //     right_coeff.push_back(0);
    //     right_coeff.push_back(0);
    //     right_coeff.push_back(0);
    //     right_coeff.push_back(0);
    // }

    // for (int i = 0; i < left_rects.size(); i++)
    // {
    //     std::cout << "\nleft rect x: " << left_rects.at(i).x;
    //     std::cout << "\nleft rect y: " << left_rects.at(i).y;
    // }
    // for (int i = 0; i < right_rects.size(); i++)
    // {
    //     std::cout << "\nright rect x: " << right_rects.at(i).x;
    //     std::cout << "\nright rect y: " << right_rects.at(i).y;
    // }

    // std::cout << "\nCoefficients:\n";
    // for (auto it = left_coeff.begin(); it != left_coeff.end(); ++it)
    //     std::cout << *it << " ";
    // std::cout << "\nCoefficients weighted:\n";
    // for (auto it = right_coeff.begin(); it != right_coeff.end(); ++it)
    //     std::cout << *it << " ";
    cv::imshow("Image", cv_ptr->image);
    cv::imshow("Warped", colored);

    lanes.header.stamp = ros::Time::now();

    lanes.left_lane.c0 = left_coeff.at(0);
    lanes.left_lane.c1 = left_coeff.at(1);
    lanes.left_lane.c2 = left_coeff.at(2);
    lanes.left_lane.c3 = left_coeff.at(3);

    lanes.right_lane.c0 = right_coeff.at(0);
    lanes.right_lane.c1 = right_coeff.at(1);
    lanes.right_lane.c2 = right_coeff.at(2);
    lanes.right_lane.c3 = right_coeff.at(3);

    cv::waitKey(3);
    return lanes;
}
