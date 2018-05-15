#include <ur10_robot/ur10_robot.h>

#include <exception>
#include <iomanip>
#include <fstream>

#include <ros/package.h>

#include <io_lib/io_lib.h>

#include <ur_kinematics/ur_kin.h>
#include <ur_kinematics/ikfast.h>

namespace ur10_
{
  void print_warning(const std::string &msg, std::ostream &out=std::cout)
  {
    out << io_::bold << io_::yellow << msg << io_::reset;
  }

  void print_info(const std::string &msg, std::ostream &out=std::cout)
  {
    out << io_::bold << io_::blue << msg << io_::reset;
  }

  void print_error(const std::string &msg, std::ostream &out=std::cerr)
  {
    out << io_::bold << io_::red << msg << io_::reset;
  }

  arma::mat quat2rotm(const arma::vec &quat)
  {
    double qw=quat(0), qx=quat(1), qy=quat(2), qz=quat(3);

    arma::mat rotm;
    rotm = {{1 - 2*qy*qy - 2*qz*qz,      2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw},
  	        {    2*qx*qy + 2*qz*qw,  1 - 2*qx*qx - 2*qz*qz,      2*qy*qz - 2*qx*qw},
  	        {    2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,  1 - 2*qx*qx - 2*qy*qy}};

    return rotm;
  }

  Robot::Robot():spinner(3)
  {
    parseConfigFile();

    if (use_sim_time) print_warning("use_sim_time is set!\n");

    if((reverse_port <= 0) or (reverse_port >= 65535))
    {
			print_warning("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 50001\n");
			reverse_port = 50001;
		}

    interface.reset(new UrInterface(host, reverse_port));

  	//spinner.start();

    if (read_wrench_from_topic) wrench_sub = n.subscribe(this->read_wrench_topic, 1, &Robot::readWrenchCallback, this);

    // this->tfListener.reset(new tf2_ros::TransformListener(this->tfBuffer));

    mode = ur10_::Mode::POSITION_CONTROL;

    printRobotStateThread_running = false;
    logging_on = false;
    cycle = interface->robot_.getServojTime(); // 0.008; // control cycle of 8 ms
    timer.tic();

    ros::Duration(3.0).sleep(); // needed to let UR initialize

    waitNextCycle();
    time_offset = rSt.time;
  }

  void Robot::parseConfigFile()
  {
    std::string params_path = ros::package::getPath("ur10_robot") + "/config/ur10_config.yml";
    param_::Parser parser(params_path);

    if (!parser.getParam("read_wrench_topic", read_wrench_topic)) read_wrench_from_topic = false;

    if (!parser.getParam("base_frame", base_frame))
      throw std::ios_base::failure("ur10_::Robot::getParam(base_frame) could not be retrieved.\n");

    if (!parser.getParam("tool_frame", tool_frame))
      throw std::ios_base::failure("ur10_::Robot::getParam(tool_frame) could not be retrieved.\n");

    if (!parser.getParam("use_sim_time", use_sim_time)) use_sim_time = false;

    if (!parser.getParam("host", host)) host = "localhost";

    if (!parser.getParam("reverse_port", reverse_port)) reverse_port = 50001;

  }

  bool Robot::isOk() const
  {
    return (!interface->robot_.sec_interface_->robot_state_->isEmergencyStopped() &&
            !interface->robot_.sec_interface_->robot_state_->isProtectiveStopped() &&
            interface->robot_.sec_interface_->robot_state_->isReady());
  }

  Robot::~Robot()
  {
    if (printRobotState_thread.joinable()) printRobotState_thread.join();
    interface->halt();
  }

  void Robot::setMode(const ur10_::Mode &mode)
  {
    if (this->getMode() == mode) return;

    switch (mode)
    {
      case ur10_::Mode::FREEDRIVE_MODE:
        this->freedrive_mode();
        break;
      case ur10_::Mode::FORCE_MODE:
        // this->force_mode();
        break;
      case ur10_::Mode::POSITION_CONTROL:
        this->position_control_mode();
        break;
      case ur10_::Mode::VELOCITY_CONTROL:
        this->velocity_control_mode();
        break;
    }
    this->mode = mode;
  }

  void Robot::position_control_mode()
  {
    if (this->getMode() != ur10_::Mode::POSITION_CONTROL)
    {
      this->stopj(2.0); // just stop the robot if it was moving
      this->mode = ur10_::Mode::POSITION_CONTROL;
    }
  }

  void Robot::velocity_control_mode()
  {
    if (this->getMode() != ur10_::Mode::VELOCITY_CONTROL)
    {
      this->stopj(2.0); // just stop the robot if it was moving
      this->mode = ur10_::Mode::VELOCITY_CONTROL;
    }
  }

  void Robot::freedrive_mode()
  {
    if (this->getMode() != ur10_::Mode::FREEDRIVE_MODE)
    {
      command_mode("freedrive_mode()\n");
      this->mode = ur10_::Mode::FREEDRIVE_MODE;
    }
  }

  void Robot::end_freedrive_mode()
  {
    urScript_command("end_freedrive_mode()\n");
    this->setMode(ur10_::Mode::POSITION_CONTROL);
  }

  void Robot::movej(const arma::vec &q, double a, double v, double t, double r)
  {
    std::ostringstream out;
    out << "movej(" << print_vector(q) << "," << a << "," << v << "," << t << "," << r << ")\n";
    urScript_command(out.str());
  }

  void Robot::movel(const arma::vec &p, double a, double v, double t, double r)
  {
    std::ostringstream out;
    out << "movel(p" << print_vector(p) << "," << a << "," << v << "," << t << "," << r << ")\n";
    urScript_command(out.str());
  }

  void Robot::speedj(arma::vec dq, double a, double t)
  {
    std::ostringstream out;
    out << "speedj(" << print_vector(dq) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")\n";
    urScript_command(out.str());
  }

  void Robot::speedl(arma::vec dp, double a, double t)
  {
    std::ostringstream out;
    out << "speedl(" << print_vector(dp) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")\n";
    urScript_command(out.str());
  }

  void Robot::stopj(double a)
  {
    std::ostringstream out;
    out << "stopj(" << a << ")\n";
    urScript_command(out.str());
  }

  void Robot::stopl(double a)
  {
    std::ostringstream out;
    out << "stopl(" << a << ")\n";
    urScript_command(out.str());
  }

  // void Robot::set_gravity(const arma::vec &g)
  // {
  //   std::ostringstream out;
  //   out << "set_gravity(" << print_vector(g) << ")\n";
  //   urScript_command(out.str());
  // }
  //
  // void Robot::set_payload(double m, const arma::vec &CoG)
  // {
  //   std::ostringstream out;
  //   out << "set_payload(" << m << "," << print_vector(CoG) << ")\n";
  //   urScript_command(out.str());
  // }
  //
  // void Robot::set_payload_cog(const arma::vec &CoG)
  // {
  //   std::ostringstream out;
  //   out << "set_payload_cog(" << print_vector(CoG) << ")\n";
  //   urScript_command(out.str());
  // }
  //
  // void Robot::set_payload_mass(double m)
  // {
  //   std::ostringstream out;
  //   out << "set_payload_mass(" << m << ")\n";
  //   urScript_command(out.str());
  // }
  //
  // void Robot::set_tcp(const arma::vec &pose)
  // {
  //   std::ostringstream out;
  //   out << "set_tcp(p" << print_vector(pose) << ")\n";
  //   urScript_command(out.str());
  // }

  void Robot::sleep(double t)
  {
    std::ostringstream out;
    out << "sleep(" << t << ")\n";
    urScript_command(out.str());
  }

  void Robot::powerdown()
  {
    urScript_command("powerdown()\n");
  }

  void Robot::waitNextCycle()
  {
    std::unique_lock<std::mutex> robotState_lck(this->robotState_mtx);

    update();

    if (logging_on) logDataStep();

  }

  void Robot::update()
  {
    // interface->robot_.rt_interface_->rt_run_cond_.notify_all();

		// std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
		// std::unique_lock<std::mutex> locker(msg_lock);

		// while (!interface->robot_.rt_interface_->robot_state_->getControllerUpdated())
		// {
		// 	// interface->rt_msg_cond_.wait(locker);
		// }

    rSt.time = ros::Time::now().toSec();

    rSt.q = interface->robot_.rt_interface_->robot_state_->getQActual();
		for (unsigned int i = 0; i < rSt.q.size(); i++)
      rSt.q[i] += interface->joint_offsets_[i];

    rSt.dq = interface->robot_.rt_interface_->robot_state_->getQdActual();
    rSt.jTorques = interface->robot_.rt_interface_->robot_state_->getIActual();
    rSt.wrench = interface->robot_.rt_interface_->robot_state_->getTcpForce();

    // Tool vector: Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    std::vector<double> tool_vector_actual = interface->robot_.rt_interface_->robot_state_->getToolVectorActual();

    // tool pose
    double rx = tool_vector_actual[3];
    double ry = tool_vector_actual[4];
    double rz = tool_vector_actual[5];
    double angle = std::sqrt(std::pow(rx,2) + std::pow(ry,2) + std::pow(rz,2));
    rSt.pos = {tool_vector_actual[0], tool_vector_actual[1], tool_vector_actual[2]};
    double cos_theta2 = std::cos(angle/2);
    double sin_theta2 = std::sin(angle/2);
    rSt.Q = {cos_theta2, rx/angle*sin_theta2, ry/angle*sin_theta2, rz/angle*sin_theta2};
    rSt.pose.submat(0,0,2,2) = quat2rotm(rSt.Q);
    rSt.pose.submat(0,3,2,3) = rSt.pos;
    rSt.pose.row(3) = arma::rowvec({0, 0, 0, 1});

    // tool velocity
    arma::vec twist = interface->robot_.rt_interface_->robot_state_->getTcpSpeedActual();
    rSt.v_lin = twist.subvec(0,2);
    rSt.v_rot = twist.subvec(3,5);

    // interface->robot_.rt_interface_->robot_state_->setControllerUpdated();


    if (read_wrench_from_topic) ros::spinOnce();

    // this->readTaskPoseCallback();
  }

  void Robot::readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    rSt.wrench << msg->wrench.force.x << msg->wrench.force.y << msg->wrench.force.z
              << msg->wrench.torque.x << msg->wrench.torque.y << msg->wrench.torque.z;
  }

  void Robot::readTaskPoseCallback()
  {
    try{
       this->transformStamped = tfBuffer.lookupTransform(this->base_frame, this->tool_frame, ros::Time(0));
       rSt.pos << this->transformStamped.transform.translation.x
               << this->transformStamped.transform.translation.y
               << this->transformStamped.transform.translation.z;

       rSt.Q << this->transformStamped.transform.rotation.w
             << this->transformStamped.transform.rotation.x
             << this->transformStamped.transform.rotation.y
             << this->transformStamped.transform.rotation.z;

       rSt.pose.submat(0,0,2,2) = quat2rotm(rSt.Q);
       rSt.pose.submat(0,3,2,3) = rSt.pos;
       rSt.pose.row(3) = arma::rowvec({0, 0, 0, 1});

       rSt.time = ros::Time::now().toSec();

     }
     catch (tf2::TransformException &ex) {
       ROS_WARN("%s",ex.what());
       // ros::Duration(1.0).sleep();
     }

  }

  void Robot::printRobotState(std::ostream &out) const
  {
    out << "===============================\n";
    out << "=======   Robot state  ========\n";
    out << "===============================\n";
    out << "time: " << getTime() << " sec\n";
    out << "joint pos: " << getJointPosition().t()*180/arma::datum::pi << "\n";
    out << "joint vel: " << getJointVelocity().t() << "\n";
    out << "joint Torques: " << getJointTorque().t() << "\n";

    out << "Cart pos: " << getTaskPosition().t() << "\n";
    out << "Cart orient: " << getTaskOrientation().t() << "\n";
    out << "Task velocity: " << getTaskVelocity().t() << "\n";
    out << "Wrench: " << getTaskWrench().t() << "\n";

    out << "===============================\n";
    out << "===============================\n";
  }

  void Robot::printRobotStateThreadFun(double freq, std::ostream &out)
  {
    ros::Rate loop_rate(freq);
    std::unique_lock<std::mutex> robotState_lck(this->robotState_mtx, std::defer_lock);

    while (printRobotStateThread_running)
    {
      robotState_lck.lock();
      this->printRobotState(out);
      robotState_lck.unlock();
      loop_rate.sleep();
    }
  }

  void Robot::launch_printRobotStateThread(double freq, std::ostream &out)
  {
    if (printRobotStateThread_running == false)
    {
      printRobotStateThread_running = true;
      printRobotState_thread =  std::thread(&Robot::printRobotStateThreadFun, this, freq, std::ref(out));
    }
  }

  void Robot::stop_printRobotStateThread()
  {
    printRobotStateThread_running = false;
    if (printRobotState_thread.joinable()) printRobotState_thread.join();
  }


  void Robot::load_URScript(const std::string &path_to_URScript)
  {
    try{ io_::readFile(path_to_URScript, ur_script); }
    catch(std::exception &e) { throw std::ios_base::failure(std::string("ur10_::Robot::load_URScript: failed to read \""+path_to_URScript+"\"...\n")); }
  }

  void Robot::execute_URScript()
  {
    urScript_command(this->ur_script);
  }

  void Robot::getRobotState(RobotState &robotState) const
  {
    robotState = this->rSt;
  }

  void Robot::command_mode(const std::string &mode)
  {
    std::string cmd;
    cmd = "def command_mode():\n\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\nend\n";
    //std::cout << "cmd=\n" << cmd << "\n";
    urScript_command(cmd);
  }

  void Robot::startLogging()
  {
    logging_on = true;
  }

  void Robot::stopLogging()
  {
    logging_on = false;
  }

  void Robot::saveLoggedData(const std::string filename, bool binary, int precision)
  {
    std::ofstream out(filename, std::ios::out);
    if (!out) throw std::ios_base::failure("Couldn't create file \"" + filename + "\"...\n");

    io_::write_mat(log_data.Time, out, binary, precision);
    io_::write_mat(log_data.q_data, out, binary, precision);
    io_::write_mat(log_data.dq_data, out, binary, precision);
    io_::write_mat(log_data.pos_data, out, binary, precision);
    io_::write_mat(log_data.Q_data, out, binary, precision);
    io_::write_mat(log_data.V_data, out, binary, precision);
    io_::write_mat(log_data.wrench_data, out, binary, precision);
    io_::write_mat(log_data.jTorques_data, out, binary, precision);

    out.close();
  }

  void Robot::logDataStep()
  {
    log_data.Time = arma::join_horiz(log_data.Time, arma::mat({getTime()}));
    log_data.q_data = arma::join_horiz(log_data.q_data, getJointPosition());
    log_data.dq_data = arma::join_horiz(log_data.dq_data, getJointVelocity());
    log_data.pos_data = arma::join_horiz(log_data.pos_data, getTaskPosition());
    log_data.Q_data = arma::join_horiz(log_data.Q_data, getTaskOrientation());
    log_data.V_data = arma::join_horiz(log_data.V_data, getTaskVelocity());
    log_data.wrench_data = arma::join_horiz(log_data.wrench_data, getTaskWrench());
    log_data.jTorques_data = arma::join_horiz(log_data.jTorques_data, getJointTorque());
  }

  void Robot::setJointTrajectory(const arma::vec &qT, double duration)
  {
    this->movej(qT, 4.0, 3.5, duration);
    ros::Duration(duration).sleep();
    this->waitNextCycle();
  }

  void Robot::setJointPosition(const arma::vec &qd)
  {
    this->movej(qd, 1.4, 1.0, this->cycle);
  }

  void Robot::setJointVelocity(const arma::vec &dqd)
  {
    this->speedj(dqd, 6.0, this->cycle);
  }

  void Robot::setTaskPose(const arma::mat &pose)
  {
    const arma::vec p;
    //convertPose2PosAngles(pose, p);
    this->movel(p, 1.2, 1.0, this->cycle);
  }

  void Robot::setTaskVelocity(const arma::vec &Twist)
  {
    this->speedl(Twist, arma::max(arma::abs((Twist-getTaskVelocity()))/this->cycle), this->cycle);
  }

  arma::vec Robot::getTaskWrench() const
  {
    arma::mat T_robot_ee = this->getTaskPose();
    arma::vec wrench(6);

    wrench.subvec(0,2) = T_robot_ee.submat(0,0,2,2)*rSt.wrench.subvec(0,2);
    wrench.subvec(3,5) = T_robot_ee.submat(0,0,2,2)*rSt.wrench.subvec(3,5);

    return wrench;
  }

} // namespace ur10_
