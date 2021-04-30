#include"robot_ball_chaser/robot_ball_chaser.hpp"

namespace robot_ball_chaser
{
    RobotBallChaser::RobotBallChaser(ros::NodeHandle& nh, const std::string nodeName)
    :nodeHandle_(nh),nodeName_(nodeName)
    {
        //get node details from yaml config file
        std::string serviceName, camera_topic, velocity_topic;
        int qSize;
        if((!nodeHandle_.getParam("service_name", serviceName)) || (!nodeHandle_.getParam("camera_topic",camera_topic))
           || (!nodeHandle_.getParam("velocity_topic",velocity_topic)) || (!nodeHandle_.getParam("qeue_size",qSize)))
        {
            ROS_ERROR("Could not find params for node : %s", nodeName_.c_str());
        }

        //create camera subscriber
        subscriber_ = nodeHandle_.subscribe(camera_topic, qSize, &RobotBallChaser::ImageCallback , this);

        //create robot cmd publisher
        robotCmdPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(velocity_topic,qSize);

        //create process image server client
        imageClient_ = nodeHandle_.serviceClient<robot_ball_chaser::ballChase>(serviceName);
    }

    void RobotBallChaser::ImageCallback(const sensor_msgs::Image &img)
    {
         //create a service
        robot_ball_chaser::ballChase service;

        //fill the service input
        service.request.image = img;
        if(imageClient_.call(service))
        {
            //update robotCmd_ with service response.
            robotCmd_.linear.x = service.response.motor_control.linear.x;
            robotCmd_.angular.z = service.response.motor_control.angular.z;

            //Publish motor control data of robotCmd_.
            robotCmdPublisher_.publish(robotCmd_);
            
            ROS_INFO("Service called successfully!");
        }
        else
        {
            ROS_ERROR("Failed to call service ");
        }
    }
}