#include"robot_ball_chaser/process_image_service.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    //select Ros node name
    const std::string nodeName("process_image_server");

    //Init Node with its name
    ros::init(argc, argv, nodeName);

    //create node handle
    ros::NodeHandle nh("~");

    //create object node
    process_image_service::ProcessImageService imgServer(nh, nodeName);
    ros::spin();

    return 0;
}