#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "boost/shared_ptr.hpp"
#include "opencv2/imgcodecs.hpp"
#include "autonomous_msgs/LaneInfo.h"



class LaneFinder
{
  private:
    /* data */
  public:
    LaneFinder(/* args */);
    ~LaneFinder();
    autonomous_msgs::LaneInfo findLanes(cv_bridge::CvImagePtr cv_ptr, autonomous_msgs::LaneInfo lanes, float k1, float k2);
};
