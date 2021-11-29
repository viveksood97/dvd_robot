/**
 *  @copyright MIT License, © 2021 Vivek Sood
 *  @file    avoid_walls.cpp
 *  @author  Vivek Sood
 *  @date    11/27/2021
 *  @version 1.0
 *  @brief   Class To handle wall avoidance mechanism
 */


#include <dvd_robot/avoid_walls.hpp>

AvoidWalls::AvoidWalls(ros::NodeHandle nh) {
  this->nh = nh;
  this->vel_publisher = this->nh.advertise<geometry_msgs::Twist>(
  "/cmd_vel", 20);

  this->lidar_subscriber = this->nh.subscribe("/scan",
  20, &AvoidWalls::lidar_callback, this);

  ros::spinOnce();
}

AvoidWalls::~AvoidWalls() {
}

bool AvoidWalls::not_obstacle(const std::vector<float>& laserscan_data_range) {
  int range = 20;
  int left = 0 + range;
  int right = 360 - range;

  for (int i=0; i != left; ++i) {
    if (laserscan_data_range[i] <= this->thresh) {
      return false;
    }
  }

  for (int i=359; i != right; --i) {
    if (laserscan_data_range[i] <= this->thresh) {
      return false;
    }
  }
  return true;
}

void AvoidWalls::lidar_callback(
  const sensor_msgs::LaserScan::ConstPtr& laserscan_msg) {
  int range = 20;
  int front_ind = 0;
  int left_ind = 0 + range;
  int right_ind = 360 - range;
  this->thresh = 1;
  ROS_INFO_STREAM("Laser data::"
                  << " Left: " << laserscan_msg->ranges[left_ind]
                  << " Front: " << laserscan_msg->ranges[front_ind]
                  << " Right: " << laserscan_msg->ranges[right_ind]);

  geometry_msgs::Twist velocity_cmd;

  // check for obstacles:
  if (this->not_obstacle(laserscan_msg->ranges)) {
    // move forward
    velocity_cmd.linear.x = 0.5;
    velocity_cmd.angular.z = 0.0;
  } else {
    ROS_WARN_STREAM("Obstacle Detected: Turning Manuevers Initiated");
    velocity_cmd.linear.x = 0.0;
    velocity_cmd.angular.z = 1.0;
  }

  this->vel_publisher.publish(velocity_cmd);
}
