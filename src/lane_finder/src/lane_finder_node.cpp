#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "lane_finder.h"

LaneFinder lane_finder;
autonomous_msgs::LaneInfo lanes;

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
    
    lanes = lane_finder.findLanes(cv_ptr, lanes);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_finder_node");
    ros::NodeHandle nh;
    ros::Rate r(20);
    ros::Subscriber camera_sub = nh.subscribe("/camera/image", 1, cameraCallback);
    ros::Publisher lanes_pub = nh.advertise<autonomous_msgs::LaneInfo>("/lanes", 10);

    while (ros::ok())
    {
        lanes_pub.publish(lanes);
        ros::spinOnce();
        r.sleep();
    }
    

    return 0;
}
