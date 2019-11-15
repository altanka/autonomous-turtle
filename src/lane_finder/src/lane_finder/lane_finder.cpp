#include "lane_finder.h"

LaneFinder::LaneFinder(/* args */)
{
}

LaneFinder::~LaneFinder()
{
}

float LaneFinder::findLanes(cv_bridge::CvImagePtr cv_ptr)
{

    cv::Point2f srcRect[4];
    srcRect[0] = cv::Point2f((cv_ptr->image.cols)*0.30f, (cv_ptr->image.rows)/2);
    srcRect[1] = cv::Point2f((cv_ptr->image.cols)*0.70f, (cv_ptr->image.rows)/2);
    srcRect[2] = cv::Point2f(0.f, cv_ptr->image.rows-20);
    srcRect[3] = cv::Point2f((cv_ptr->image.cols), cv_ptr->image.rows-20);
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


    cv::imshow("Warped", warp_dst);
    // cv::imshow("Image", cv_ptr->image);
    cv::waitKey(3);
    return 0;
}
