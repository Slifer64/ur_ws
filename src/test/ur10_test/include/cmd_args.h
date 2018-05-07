#ifndef UR10_TEST_CMD_ARGS_H
#define UR10_TEST_CMD_ARGS_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <param_lib/param_lib.h>

#include <armadillo>

struct CMD_ARGS
{
  double ctr_cycle_rate;
  double print_robotState_rate;

  bool move_to_init_config_flag;
  bool move_to_goal_config_flag;
  arma::vec init_joint_pos;
  arma::vec goal_joint_pos;

  double joint_vel;
  double joint_accel;
  double Cart_vel;
  double Cart_accel;

  arma::mat Md;
  arma::mat Kd;
  arma::mat Dd;

  CMD_ARGS();
  bool parse_cmd_args();
  void print(std::ostream &out=std::cout) const;

private:
  std::string path_to_config_file;
  std::unique_ptr<as64_::param_::Parser> parser;
};

#endif // UR10_TEST_CMD_ARGS_H
