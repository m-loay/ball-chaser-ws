# ball-chaser-ws
In this project a mobile robot is chasing a ball inside a Gazebo envionment.Both eviornment and robot are customized
for this project.The robot depends on camera to detect the ball and move toward it.

<img src="images/course_code_structure.png" width="779" height="414" />

## Directory Structure
```                   
    .ball-chaser-ws                          # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── roboWorld.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──                                                        
```
## Project Description

In this project, I created two ROS packages inside my `ball-chaser-ws/src`:  
1. `my_robot`:  
holds a robot designed with the Unified Robot Description Format, a white-colored ball, and a gazebo world.  
2. `ball_chaser`:  
contains two C++ ROS nodes (`drive_bot` & `process_image`) to interact with the robot and make it chase the white ball.  
    - `drive_bot`:  
    provides a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities.  
    The service publishes to the wheel joints and return back the requested velocities.  
    - `process_image`  
    reads my robot’s camera image, analyzes it to determine the presence and position of a white ball.  
    If a white ball exists in the image, my node requests a service via a client to drive the robot towards it.  

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* ROS  >= (Kinetic/Melodic/Noetic)
  * For all platform and OS [Click here for installation instructions](http://wiki.ros.org/ROS/Installation)

## Basic Build Instructions

1. Clone this repo.
2. Inside cloned folder `catkin_make`
3. Then source the workspace: `source devel/setup.bash`
4. Then run the launch files: `roslaunch my_robot world.launch` and `roslaunch ball_chaser ball_chaser.launch`.