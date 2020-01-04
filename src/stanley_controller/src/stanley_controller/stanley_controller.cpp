#include "stanley_controller.h"
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

StanleyController::StanleyController(float K, float K_soft)
{
    StanleyController::K = K;
    StanleyController::K_soft = K_soft;
}

StanleyController::~StanleyController()
{
}

double StanleyController::calculateSteeringAngle(const autonomous_msgs::LaneInfo::ConstPtr& msg, float vehicle_speed)
{
    c0_l_ = msg->left_lane.c0;
    c1_l_ = msg->left_lane.c1;
    c2_l_ = msg->left_lane.c2;
    c3_l_ = msg->left_lane.c3;
    c0_r_ = msg->right_lane.c0;
    c1_r_ = msg->right_lane.c1;
    c2_r_ = msg->right_lane.c2;
    c3_r_ = msg->right_lane.c3;
    double yaw_error = (c1_l_ + c1_r_) / 2;
    // yaw_error = 0;
    double cross_track_error = atan2(K * (c0_l_ + c0_r_), vehicle_speed + K_soft)*(180/M_PI);

    std::cout << "\n----------------------------------------------------\n";
    std::cout << "\nStanley c0_l : " << c0_l_;
    std::cout << "\nStanley c0_r : " << c0_r_;
    std::cout << "\nStanley c1_l : " << c1_l_;
    std::cout << "\nStanley c1_r : " << c1_r_;
    std::cout << "\nStanley Cross Track Error : " << cross_track_error;
    std::cout << "\nStanley Yaw Error         : " << yaw_error;
    double steering_angle = cross_track_error + yaw_error;
    std::cout << "\nStanley Error             : " << steering_angle << "\n";
    steering_angle = steering_angle > 15 ? 15 : steering_angle;
    steering_angle = steering_angle < -15 ? -15 : steering_angle;
    return steering_angle;
}