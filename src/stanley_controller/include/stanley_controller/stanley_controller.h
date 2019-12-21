#include "autonomous_msgs/LaneInfo.h"

class StanleyController
{
  private:
    float c0_l_;
    float c1_l_;
    float c2_l_;
    float c3_l_;
    float c0_r_;
    float c1_r_;
    float c2_r_;
    float c3_r_;
    float vehicle_speed_;

  public:
    float K;
    float K_soft;
    StanleyController(float K, float K_soft);
    ~StanleyController();
    double calculateSteeringAngle(const autonomous_msgs::LaneInfo::ConstPtr& msg, float vehicle_speed);
};
