#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>
#include <thread>
#include <mutex>

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
  ros::init(argc, argv, "test_force_mode_node");
  ros::NodeHandle nh_("~");

  // ===========  Read params from yml file  ==================
  double print_robotState_rate;
  arma::vec task_frame, selection_vector, wrench, limits;
  int type;
  std::string path_to_config_file = ros::package::getPath("ur10_test")+ "/config/test_force_mode_config.yml";
  as64_::param_::Parser parser(path_to_config_file);
  if (!parser.getParam("print_robotState_rate", print_robotState_rate)) print_robotState_rate = 4;
  if (!parser.getParam("task_frame", task_frame)) task_frame = arma::vec().zeros(6);
  if (!parser.getParam("selection_vector", selection_vector)) selection_vector = arma::vec().zeros(6);
  if (!parser.getParam("wrench", wrench)) wrench = arma::vec().zeros(6);
  if (!parser.getParam("limits", limits)) limits = arma::vec().zeros(6);
  if (!parser.getParam("type", type)) type = 2;

  // ===========  Create robot  ==================
  std::shared_ptr<ur10_::Robot> robot;
  robot.reset(new ur10_::Robot());

  // ===========  Define control cycle  ==================
  ros::Rate loop_rate(125); // loop at 125 Hz

  // ===========  Launch thread for printing  ==================
  robot->launch_printRobotStateThread(print_robotState_rate);

  std::cout << "Entering force_mode...\n";
  //robot->force_mode({0,0,0,0,0,0},{1,1,1,1,1,1},{2,2,2,0,0,0},2,{0.5,0.5,0.5,0.25,0.25,0.25});
  robot->force_mode(task_frame, selection_vector, wrench, type, limits);
  // robot->freedrive_mode();

  while (ros::ok())
  {
    robot->waitNextCycle();
    loop_rate.sleep();
  }

  robot->end_force_mode();
  // robot->end_freedrive_mode();

  robot->stop_printRobotStateThread();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
