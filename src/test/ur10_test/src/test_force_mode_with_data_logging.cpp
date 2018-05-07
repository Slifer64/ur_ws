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

std::shared_ptr<ur10_::Robot> robot;
bool exit_ctrl_loop;

void keyboardCtrlThreadFun()
{
  while (true)
  {
    char key = as64_::io_::getch();
    switch (key)
    {
      case 'n':
        std::cout << as64_::io_::green <<  "Start data logging...\n" << as64_::io_::reset;
        robot->startLogging();
        break;
      case 'm':
        std::cout << as64_::io_::green << "Stop data logging...\n" << as64_::io_::reset;
        robot->stopLogging();
        break;
      case 's':
        std::cout << as64_::io_::green << "Exiting control loop...\n" << as64_::io_::reset;
        exit_ctrl_loop = true;
        return;
    }
  }
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "test_force_mode_node");
  ros::NodeHandle nh_("~");

  std::thread keyboard_ctrl_thread(keyboardCtrlThreadFun);

  // ===========  Read params from yml file  ==================
  std::string path_to_config_file = ros::package::getPath("ur10_test")+ "/config/test_force_mode_with_data_logging_config.yml";
  as64_::param_::Parser parser(path_to_config_file);

  double print_robotState_rate;
  arma::vec task_frame, selection_vector, wrench, limits;
  int type;
  if (!parser.getParam("print_robotState_rate", print_robotState_rate)) print_robotState_rate = 4;
  if (!parser.getParam("task_frame", task_frame)) task_frame = arma::vec().zeros(6);
  if (!parser.getParam("selection_vector", selection_vector)) selection_vector = arma::vec().zeros(6);
  if (!parser.getParam("wrench", wrench)) wrench = arma::vec().zeros(6);
  if (!parser.getParam("limits", limits)) limits = arma::vec().zeros(6);
  if (!parser.getParam("type", type)) type = 2;

  bool data_logging;
  bool binary;
  int precision;
  std::string data_filename;
  if (!parser.getParam("data_logging", data_logging)) data_logging = true;
  if (!parser.getParam("binary", binary)) binary = true;
  if (!parser.getParam("precision", precision)) precision = 6;
  if (!parser.getParam("data_filename", data_filename)) data_filename = ros::package::getPath("ur10_test")+ "/data/logged_data";

  // ===========  Create robot  ==================
  robot.reset(new ur10_::Robot());

  // ===========  Define control cycle  ==================
  ros::Rate loop_rate(125); // loop at 125 Hz

  // ===========  Launch thread for printing  ==================
  robot->launch_printRobotStateThread(print_robotState_rate);

  std::cout << "Entering force_mode...\n";
  //robot->force_mode({0,0,0,0,0,0},{1,1,1,1,1,1},{2,2,2,0,0,0},2,{0.5,0.5,0.5,0.25,0.25,0.25});
  robot->force_mode(task_frame, selection_vector, wrench, type, limits);
  // robot->freedrive_mode();

  exit_ctrl_loop = false;

  while (ros::ok() && !exit_ctrl_loop)
  {
    robot->waitNextCycle();
    loop_rate.sleep();
  }

  robot->end_force_mode();
  // robot->end_freedrive_mode();

  std::cout << "Saving logged data to file: \"" << data_filename << "\"\n";
  robot->saveLoggedData(data_filename, binary, precision);

  robot->stop_printRobotStateThread();

  keyboard_ctrl_thread.join();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
