#include"robot_ball_chaser/robot_ball_chaser.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    //select Ros node name
    const std::string nodeName("robot_ball_chaser_node");

    //Init Node with its name
    ros::init(argc, argv, nodeName);

    //create node handle
    ros::NodeHandle nh("~");

    //create object node
    robot_ball_chaser::RobotBallChaser robotChaser(nh, nodeName);
    ros::spin();

    return 0;
}