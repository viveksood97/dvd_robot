/**
 *  @copyright MIT License, © 2021 Vivek Sood
 *  @file    avoid_walls.hpp
 *  @author  Vivek Sood
 *  @date    11/27/2021
 *  @version 1.0
 *  @brief   Class To handle wall avoidance mechanism
 */


#ifndef INCLUDE_DVD_ROBOT_OBSTACLE_AVOIDANCE_HPP_
#define INCLUDE_DVD_ROBOT_OBSTACLE_AVOIDANCE_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

class AvoidWalls {
 public:
  /**
   * @brief Construct a new Obstacle Avoidance object
   * 
   * @param nh 
   */
  explicit AvoidWalls(ros::NodeHandle nh);

  /**
   * @brief Destroy the Obstacle Avoidance object
   * 
   */
  ~AvoidWalls();

 private:
  ros::NodeHandle nh;

  /**
   * @brief publisher object for the /cmd_vel topic
   * 
   */
  ros::Publisher vel_publisher;

  /**
   * @brief subsriber to the /sensor_msgs/LaserScan topic
   * 
   */
  ros::Subscriber lidar_subscriber;

  /**
   * @brief minimum distance of the bot from the obstacle
   * 
   */
  float thresh = 0.6;  // in metres

  int turn_count = 0;
  int MAX_ONE_SIDE_TURN_COUNT = 100;


  /**
   * @brief Callback method for the laserscan subscriber
   * 
   * @param laserscan_data laserscan topic message
   */
  void lidar_callback(
            const sensor_msgs::LaserScan::ConstPtr& laserscan_msg);

  /**
   * @brief to check if there
   * 
   * @param laserscan_data_range distances of the obstacles at every angle from 0-360
   * @param angle_range arc angle to get the area that should be obstacle free in either side
   *                    if angle_range=10:
   *                    obstacles will be checked in range: 10 deg left and 10 deg right of the bot 
   * @return true if the path is clear and there are no obstacles with a certain range 
   * @return false if obstacle/s is/are there within a certain range
   */
  bool not_obstacle(const std::vector<float>& laserscan_data_range);
};

#endif  // INCLUDE_DVD_ROBOT_OBSTACLE_AVOIDANCE_HPP_
