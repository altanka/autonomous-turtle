#include "stanley_controller.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

ros::Publisher steering_pub;
StanleyController stanley_controller(1, 1);
float vehicle_speed;

void lanesCallback(const autonomous_msgs::LaneInfo::ConstPtr& msg)
{
    std_msgs::Float32 steering_angle;
    steering_angle.data = stanley_controller.calculateSteeringAngle(msg, vehicle_speed);
    steering_pub.publish(steering_angle);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    vehicle_speed = 100*std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                              msg->twist.twist.linear.y * msg->twist.twist.linear.y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stanley_controller_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber lanes_sub = nh.subscribe("/lanes", 10, lanesCallback);
    steering_pub = nh.advertise<std_msgs::Float32>("/set_point", 10);
    ros::spin();
    return 0;
}
