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
    tf2::Quaternion return_quaternion;
    return_quaternion.setRPY(0, 0, ((c1_l_ + c1_r_) / 2 + atan2(K * (c0_l_ + c0_r_), vehicle_speed + K_soft))*(180/M_PI));
    return_quaternion.normalize();
    return ((c0_l_ + c0_r_));
}