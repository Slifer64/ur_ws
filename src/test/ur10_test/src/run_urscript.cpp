/**
 * Copyright (C) 2017 as64_
 */

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>
#include <thread>
#include <mutex>
#include <chrono>

#include <armadillo>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include <io_lib/io_lib.h>
#include <param_lib/param_lib.h>
#include <ur10_robot/ur10_robot.h>


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "run_urscript_node");
  ros::NodeHandle nh_("~");

  ros::Rate loop_rate(125); // loop at 125 Hz

  // ===========  Read params from yml file  ==================
  std::string path_to_urscript_file;
  double print_robotState_rate;
  std::string path_to_config_file = ros::package::getPath("ur10_test")+ "/config/run_urscript_config.yml";
  as64_::param_::Parser parser(path_to_config_file);
  if (!parser.getParam("path_to_urscript_file", path_to_urscript_file)) throw std::ios_base::failure("Failed to read the config file\n");
  if (!parser.getParam("print_robotState_rate", print_robotState_rate)) print_robotState_rate=1;

  // ===========  Create robot  ==================
  std::shared_ptr<ur10_::Robot> robot;
  robot.reset(new ur10_::Robot());

  // ===========  Launch thread for printing  ==================
  robot->launch_printRobotStateThread(print_robotState_rate);

  // ===========  Load and run urscript  ==================
  robot->load_URScript(path_to_urscript_file);
  robot->execute_URScript();

  while (ros::ok())
  {
    robot->waitNextCycle();
    loop_rate.sleep();
  }

  robot->stop_printRobotStateThread();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
