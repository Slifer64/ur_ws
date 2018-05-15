/*
 * ur_driver.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_modern_driver/ur_ros_wrapper.h"


int main(int argc, char **argv)
{
	bool use_sim_time = false;
	std::string host;
	int reverse_port = 50001;

	ros::init(argc, argv, "ur_driver");
	ros::NodeHandle nh;
	if (ros::param::get("use_sim_time", use_sim_time)) {
		print_warning("use_sim_time is set!!");
	}
	if (!(ros::param::get("~robot_ip_address", host))) {
		if (argc > 1) {
			print_warning(
					"Please set the parameter robot_ip_address instead of giving it as a command line argument. This method is DEPRECATED");
			host = argv[1];
		} else {
			print_fatal(
					"Could not get robot ip. Please supply it as command line parameter or on the parameter server as robot_ip");
			exit(1);
		}

	}
	if ((ros::param::get("~reverse_port", reverse_port))) {
		if((reverse_port <= 0) or (reverse_port >= 65535)) {
			print_warning("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 50001");
			reverse_port = 50001;
		}
	} else
		reverse_port = 50001;

	RosWrapper interface(host, reverse_port);

	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::waitForShutdown();

	interface.halt();

	exit(0);
}
