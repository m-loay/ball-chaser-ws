#include"robot_ball_chaser/process_image_service.hpp"

namespace process_image_service
{
    ProcessImageService::ProcessImageService(ros::NodeHandle& nh, const std::string nodeName)
    :nodeHandle_(nh),nodeName_(nodeName)
    {
        //get node details from yaml config file
        std::string serviceName;
        if(!nodeHandle_.getParam("service_name",serviceName))
        {
            ROS_ERROR("Could not find params for node : %s", nodeName_.c_str());
        }

        //create a service 
        imageServer_ = nodeHandle_.advertiseService(serviceName, &ProcessImageService::ProcessImageCallback,this);
    }

    bool ProcessImageService::ProcessImageCallback(robot_ball_chaser::ballChaseRequest &request,
                                                   robot_ball_chaser::ballChaseResponse &response)
    {
        const int white_pixel = 255;

        bool is_ball_in_frame = false;
        int col_index = -1;
        
        // TODO: Loop through each pixel in the image and check if there's a bright white one
        for (int i = 0; i < (request.image.height * request.image.step); i=i+3)
        {
            if(request.image.data[i] == white_pixel && request.image.data[i+1] == white_pixel && request.image.data[i+2] == white_pixel)
            {
                is_ball_in_frame = true;
                col_index = static_cast<int>(i/3) % request.image.width;
                break;
            }
        }
        
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        if(is_ball_in_frame)
        {
            // Depending on the white ball position, call the drive_bot function and pass velocities to it

            //left region
            if (col_index < int(request.image.width/3))
            {
                response.motor_control.linear.x = 0.0f;
                response.motor_control.angular.z = 0.5f;
                ROS_INFO("LEFT REGION");
            }

            //forward region
            else if (col_index >= static_cast<int>(request.image.width/3) && col_index <= 2*static_cast<int>(request.image.width/3))
            {
                response.motor_control.linear.x = 0.5f;
                response.motor_control.angular.z = 0.0f;
                ROS_INFO("FORWARD REGION");
            }

            //right region
            else if (col_index > 2*static_cast<int>(request.image.width/3))
            {
                response.motor_control.linear.x = 0.0f;
                response.motor_control.angular.z = -0.5f;
                ROS_INFO("RIGHT REGION");
            }
            
            else
            {
                /* do nothing */
            }
        }
        else
        {
            // Request a stop when there's no white ball seen by the camera
            response.motor_control.linear.x = 0.0f;
            response.motor_control.angular.z = -0.0f;
            ROS_INFO("STOP");
        }
        return true;       
    }
}