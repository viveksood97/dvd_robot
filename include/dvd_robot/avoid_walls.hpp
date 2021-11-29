/**
 *  @copyright MIT License, © 2021 Vivek Sood
 *  @file    avoid_walls.hpp
 *  @author  Vivek Sood
 *  @date    11/27/2021
 *  @version 1.0
 *  @brief   Class To handle wall avoidance mechanism
 */


#ifndef INCLUDE_DVD_ROBOT_AVOID_WALLS_HPP_
#define INCLUDE_DVD_ROBOT_AVOID_WALLS_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

/// @class AvoidWalls
/// @brief Class to avoid obstacles/walls
class AvoidWalls {
 public:
  /**
  * @brief Constructor
  * @param nh node handler
  */
  explicit AvoidWalls(ros::NodeHandle nh);

  /**
  * @brief Destructor
  */
  ~AvoidWalls();

 private:
  ros::NodeHandle nh;
  ros::Publisher vel_publisher;
  ros::Subscriber lidar_subscriber;
  float thresh = 0.6;  // in meters

  /**
  * @brief callback for laser scan
  * @param laserscan_msg sensor_msgs::LaserScan::ConstPtr pointer
  * @return void
  */
  void lidar_callback(
            const sensor_msgs::LaserScan::ConstPtr& laserscan_msg);

 /**
  * @brief check for obstacles
  * @param laserscan_data_range std::vector<float> pointer
  * @return bool
  */
  bool not_obstacle(const std::vector<float>& laserscan_data_range);
};

#endif  // INCLUDE_DVD_ROBOT_AVOID_WALLS_HPP_
