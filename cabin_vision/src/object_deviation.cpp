#include "cabin_vision/object_deviation.h"
#define RED "\033[1m\033[31m"
ObjectDeviation::ObjectDeviation() : nh("~"){
    bounding_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &ObjectDeviation::BoundingBoxSub, this);
    deviation_pub = nh.advertise<geometry_msgs::PointStamped>("/cabin_vision/deviation", 1);
    
    //Load the width and heigth of the camera image
    string properties_file;
    nh.param("properties_file", properties_file, std::string(""));
    if(properties_file.size() != 0){
        if(access(properties_file.c_str(),F_OK) == -1){
            std::cout<<""<<std::endl;
            std::cout<<RED<<"\""<<properties_file<<"\""<<" does not exist!!!!"<<std::endl;
            std::cout<<""<<std::endl;
            ros::shutdown();
        }
        else{
            YAML::Node properties;
            properties = YAML::LoadFile(properties_file);
            image_width = properties["image_width"].as<int>();
            image_height = properties["image_height"].as<int>();
        }
    }
}

//Subscribe the boundingbox msg and calculate the deviation between the center of the object and the center of the image
void ObjectDeviation::BoundingBoxSub(const darknet_ros_msgs::BoundingBoxes msg){
    deviation_msg.header.stamp = ros::Time::now();
    deviation_msg.point.x = (msg.bounding_boxes[0].xmin + msg.bounding_boxes[0].xmax) / 2.0 - image_width / 2.0;
    deviation_msg.point.y = (msg.bounding_boxes[0].ymin + msg.bounding_boxes[0].ymax) / 2.0 - image_height / 2.0;
    deviation_pub.publish(deviation_msg);
}

void ObjectDeviation::Loop(){
    ros::Rate rate(50);
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "object_deviation");
    ObjectDeviation tc;
    tc.Loop();
}