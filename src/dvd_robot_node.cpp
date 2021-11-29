/**
 *  @copyright MIT License, © 2021 Vivek Sood
 *  @file    main.cpp
 *  @author  Vivek Sood
 *  @date    11/27/2021
 *  @version 1.0
 *  @brief   main file
 */



#include <ros/ros.h>
#include <std_msgs/String.h>



#include <dvd_robot/avoid_walls.hpp>

 /**
 * @brief Main function
 * @param argc number of input arguments
 * @param argv char pointer containing arguments
 * @return 0
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "dvd_robot");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Starting walker bot node.... ");
  std::unique_ptr<AvoidWalls> dvd_robot(new AvoidWalls(
                                              nh));
  ros::spin();
  return 0;
}
