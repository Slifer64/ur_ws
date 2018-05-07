#include <cmd_args.h>
#include <io_lib/io_lib.h>


CMD_ARGS::CMD_ARGS()
{
  std::string package_path = ros::package::getPath("ur10_test");
  path_to_config_file = package_path + "/config/config.yml";
  parser.reset(new as64_::param_::Parser(path_to_config_file));
}

bool CMD_ARGS::parse_cmd_args()
{
  if (!parser->getParam("ctr_cycle_rate", ctr_cycle_rate)) ctr_cycle_rate = 100.0;
  if (!parser->getParam("print_robotState_rate", print_robotState_rate)) print_robotState_rate = 1.0;

  if (!parser->getParam("move_to_init_config_flag", move_to_init_config_flag)) move_to_init_config_flag = false;
  if (!parser->getParam("move_to_goal_config_flag", move_to_goal_config_flag)) move_to_goal_config_flag = false;
  if (!parser->getParam("init_joint_pos", init_joint_pos)) init_joint_pos = arma::vec().zeros(6);
  if (!parser->getParam("goal_joint_pos", goal_joint_pos)) goal_joint_pos = arma::vec().zeros(6);

  if (!parser->getParam("joint_vel", joint_vel)) joint_vel = 0.5;
  if (!parser->getParam("joint_accel", joint_accel)) joint_accel = 0.4;
  if (!parser->getParam("Cart_vel", Cart_vel)) Cart_vel = 0.6;
  if (!parser->getParam("Cart_accel", Cart_accel)) Cart_accel = 0.3;

  arma::vec temp;
  if (!parser->getParam("Md", temp)) temp = arma::vec().ones(3);
  Md = arma::diagmat(temp);

  if (!parser->getParam("Kd", temp)) temp = arma::vec().ones(3)*100;
  Kd = arma::diagmat(temp);

  if (!parser->getParam("Dd", temp)) temp = arma::vec().ones(3)*20;
  Dd = arma::diagmat(temp);

}

void CMD_ARGS::print(std::ostream &out) const
{
  out << "ctr_cycle_rate: " << ctr_cycle_rate << "\n";
  out << "print_robotState_rate: " << print_robotState_rate << "\n";

  out << "move_to_init_config_flag: " << move_to_init_config_flag << "\n";
  out << "move_to_goal_config_flag: " << move_to_goal_config_flag << "\n";
  out << "init_joint_pos: [" << init_joint_pos << "]\n";
  out << "goal_joint_pos: [" << goal_joint_pos << "]\n";

  out << "joint_vel: " << joint_vel << "\n";
  out << "joint_accel: " << joint_accel << "\n";
  out << "Cart_vel: " << Cart_vel << "\n";
  out << "Cart_accel: " << Cart_accel << "\n";

  out << "Md: " << Md << "\n";
  out << "Kd: " << Kd << "\n";
  out << "Dd: " << Dd << "\n";

}
