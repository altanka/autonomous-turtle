#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "lane_finder.h"

LaneFinder lane_finder;

void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    lane_finder.findLanes(cv_ptr);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_finder_node");
    ros::NodeHandle nh;
    ros::Subscriber camera_sub = nh.subscribe("/camera/image", 1, cameraCallback);

    ros::spin();

    return 0;
}
