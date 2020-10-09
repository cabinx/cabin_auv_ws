#ifndef OBJECT_DEVIATION_H
#define OBJECT_DEVIATION_H
#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include <unistd.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "geometry_msgs/PointStamped.h"
using namespace std;
class ObjectDeviation{
    private:
        ros::NodeHandle nh;
        ros::Subscriber bounding_box_sub;
        ros::Publisher deviation_pub;
        int image_width, image_height;
        geometry_msgs::PointStamped deviation_msg;
    
    public:
        ObjectDeviation();
        void BoundingBoxSub(const darknet_ros_msgs::BoundingBoxes msg);
        void Loop();
};
#endif