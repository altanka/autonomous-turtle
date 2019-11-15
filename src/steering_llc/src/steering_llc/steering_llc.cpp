#include "steering_llc.h"

SteeringLLC::SteeringLLC(float K_P, float K_I, float K_D, float sampling_time, float max_vel, float min_vel)
{
    SteeringLLC::K_P_ = K_P;
    SteeringLLC::K_I_ = K_I;
    SteeringLLC::K_D_ = K_D;
    SteeringLLC::sampling_time_ = sampling_time;
    SteeringLLC::MAX_VEL_ = max_vel;
    SteeringLLC::MIN_VEL_ = min_vel;
}

SteeringLLC::~SteeringLLC()
{
}

geometry_msgs::Twist SteeringLLC::steeringControl(float setpoint, float current_steering, geometry_msgs::Twist twist)
{
    e_curr_ = setpoint - current_steering;

    // ----------------------- P -----------------------
    u_p_ = K_P_ * e_curr_;

    // ----------------------- I -----------------------
    u_i_ += K_I_ * e_curr_ * sampling_time_;

    // ----------------------- D -----------------------
    u_d_ = K_D_ * (e_curr_ - e_last_) / sampling_time_;

    u_control_ = (u_p_ + u_i_ + u_d_);

    if (u_control_ > MAX_VEL_)
    {
        u_control_ = MAX_VEL_;
        u_i_ = 0;
    }

    if (u_control_ < MIN_VEL_)
    {
        u_control_ = MIN_VEL_;
        u_i_ = 0;
    }

    std::cout << "-----------------------------\n";
    std::cout << "Error     : " << e_curr_ << "\n";
    std::cout << "U_P       : " << u_p_ << "\n";
    std::cout << "U_I       : " << u_i_ << "\n";
    std::cout << "U_D       : " << u_d_ << "\n";
    std::cout << "U_Control : " << u_control_ << "\n";

    twist.angular.z = u_control_;
    e_last_ = e_curr_;

    return twist;
}
