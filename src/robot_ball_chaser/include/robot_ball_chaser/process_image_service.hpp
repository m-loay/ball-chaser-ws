#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <robot_ball_chaser/ballChase.h>


namespace process_image_service
{
    /*!
    * Main class for the node to handle the ROS interfacing.
    */
    class ProcessImageService
    {
        private:
        /*data*/
        //! ROS node handle.
        ros::NodeHandle nodeHandle_;

        // Define a global client that can request Image services
        ros::ServiceServer imageServer_;

        //! ROS node name.
        const std::string nodeName_;

        /*!
        * ImageCallback get motor control input and publish it.
        * @param msg Image received from Camera.
        * @return void.
        */
        bool ProcessImageCallback(robot_ball_chaser::ballChaseRequest &request,
                                  robot_ball_chaser::ballChaseResponse &response);

        public:
        /*!
        * Constructor.
        * @param nodeHandle the ROS node handle.
        * @param nodeName the ROS node name.
        */
        ProcessImageService(ros::NodeHandle& nh, const std::string nodeName);

        /* @brief: Copy constructor
        * defined as DELETED for simplicity 
        * @comments: at this moment, I don't think this is necessarily a good
        * practice to wrap up node handlers, subscribers etc. into a new class,
        * because we should be very careful with the behavior of copy constructors.
        * However, if we really want to copy this class, remember to
        * copy both vars 'nodeHandle' & 'subscriber'. Do NOT initialize
        * the subscriber again from the method 'NodeHandle.subscribe()'.
        * Refer to the source code
        * http://wiki.ros.org/roscpp/Overview/NodeHandles
        * https://github.com/ros/ros_comm/blob/kinetic-devel/clients/roscpp/src/libros/node_handle.cpp
        * https://github.com/ros/ros_comm/blob/kinetic-devel/clients/roscpp/src/libros/subscriber.cpp
        */
        ProcessImageService(const ProcessImageService &) = delete;
        ProcessImageService& operator=(const ProcessImageService &) = delete;

        /*!
        * Destructor.
        */
        ~ProcessImageService() = default;
    };
}