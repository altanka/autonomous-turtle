#include "ros/ros.h"
#include "steering_llc.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"

float set_point = 0;
float current_steering;
bool initialized;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_steering = msg->orientation.z;
    initialized = true;
    std::cout << "Steering  : " << current_steering << "\n";
}

void setPointCallback(const std_msgs::Float32::ConstPtr& msg)
{
    set_point = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steering_llc_node");
    ros::NodeHandle nh("~");
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCallback);
    ros::Subscriber set_point_sub = nh.subscribe("/set_point", 1, setPointCallback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist twist_msg;
    float K_P;
    float K_I;
    float K_D;
    float rate = 50;
    float sampling_time = 1 / rate;
    float max_vel;
    float min_vel;
    ros::Rate r(rate);
    if (nh.getParam("K_P", K_P))
    {
        ROS_INFO("K_P param: %f", K_P);
    }
    else
    {
        ROS_ERROR("Failed to get param K_P");
    }
    if (nh.getParam("K_I", K_I))
    {
        ROS_INFO("K_I param: %f", K_I);
    }
    else
    {
        ROS_ERROR("Failed to get param K_I");
    }
    if (nh.getParam("K_D", K_D))
    {
        ROS_INFO("K_D param: %f", K_D);
    }
    else
    {
        ROS_ERROR("Failed to get param K_D");
    }

    if (nh.getParam("MAX_VEL", max_vel))
    {
        ROS_INFO("MAX_VEL param: %f", K_D);
    }
    else
    {
        ROS_ERROR("Failed to get param MAX_VEL");
    }

    if (nh.getParam("MIN_VEL", min_vel))
    {
        ROS_INFO("MIN_VEL param: %f", min_vel);
    }
    else
    {
        ROS_ERROR("Failed to get param MIN_VEL");
    }

    SteeringLLC steering_llc(K_P, K_I, K_D, sampling_time, max_vel, min_vel);

    while (ros::ok())
    {
        if (initialized)
        {
            twist_msg = steering_llc.steeringControl(set_point, current_steering, twist_msg);
            twist_pub.publish(twist_msg);
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
