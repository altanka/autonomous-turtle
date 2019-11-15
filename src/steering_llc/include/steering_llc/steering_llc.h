#include "geometry_msgs/Twist.h"

class SteeringLLC
{
private:
    float K_P_;
    float K_I_;
    float K_D_;
    float u_p_ = 0;
    float u_i_ = 0;
    float u_d_ = 0;
    float u_control_ = 0;
    float sampling_time_;
    float e_curr_;
    float e_last_;
    float MAX_VEL_ = 0.8;
    float MIN_VEL_ = -0.8;
public:
    SteeringLLC(float K_P, float K_I, float K_D, float sampling_time, float max_vel, float min_vel);
    ~SteeringLLC();
    geometry_msgs::Twist steeringControl(float setpoint, float current_steering, geometry_msgs::Twist twist);
};
