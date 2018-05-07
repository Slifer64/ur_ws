#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>

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

  std::string data_filename = ros::package::getPath("ur10_test")+ "/data/logged_data";
  bool data_logging = true;
  bool binary = true;
  int precision = 7;

  // ===========  Create robot  ==================
  std::shared_ptr<ur10_::Robot> robot;
  robot.reset(new ur10_::Robot());

  // ===========  Define control cycle  ==================
  ros::Rate loop_rate(125); // loop at 125 Hz

  // ===========  Launch thread for printing  ==================
  robot->launch_printRobotStateThread(1);

  while (ros::ok())
  {
    robot->waitNextCycle();
    loop_rate.sleep();
  }

  std::cout << "Stop data logging...\n";
  robot->stopLogging();

  std::cout << "Saving logged data to file: \"" << data_filename << "\"\n";
  robot->saveLoggedData(data_filename, binary, precision);

  robot->stop_printRobotStateThread();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
