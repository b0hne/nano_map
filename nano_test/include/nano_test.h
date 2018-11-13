#include <ros/ros.h>
#include <math.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <algorithm>
#include <Eigen/Dense>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <list>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>