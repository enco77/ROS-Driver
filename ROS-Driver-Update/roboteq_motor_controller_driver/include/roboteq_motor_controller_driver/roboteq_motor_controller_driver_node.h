#ifndef ROBOTEQ_MOTOR_CONTROLLER_DRIVER_MAIN_H
#define ROBOTEQ_MOTOR_CONTROLLER_DRIVER_MAIN_H

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <typeinfo>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <roboteq_motor_controller_driver/channel_values.h>
#include <roboteq_motor_controller_driver/config_srv.h>
#include <roboteq_motor_controller_driver/command_srv.h>
#include <roboteq_motor_controller_driver/maintenance_srv.h>
#include <roboteq_motor_controller_driver/emergency_stop_srv.h>
#include <roboteq_motor_controller_driver/safety_stop_srv.h>

namespace roboteq{

class RoboteqDriver
{
public:
	// RoboteqDriver()
	// {
	// 	initialize(); //constructor - Initialize
	// }
	serial::Serial ser;
	void initialize();
	void run();
	void connect();

	// ~RoboteqDriver()
	// {
	// 	if (ser.isOpen())
	// 	{
	// 		ser.close();
	// 	}
	// }
private:

	int frequency;
	float wheelbase;
	float radius;
	float gearRatio;
	float maxRPM;
	int baud_rate;
	int32_t baud;
	std::string port;
	void cmd_vel_callback(const geometry_msgs::Twist& msg);
	double calculate_right_speed(double x, double z);
	double calculate_left_speed(double x, double z);
	double to_rpm(double value);
	double max_limit(double speed);

	ros::Subscriber cmd_vel_sub;
	ros::Publisher read_publisher;
	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer maintenancesrv;
	ros::ServiceServer emergencysrv;
	ros::ServiceServer safetystopsrv;
	ros::ServiceServer multicommandsrv;

	bool configservice(roboteq_motor_controller_driver::config_srv::Request& request,     	roboteq_motor_controller_driver::config_srv::Response& response);
	bool commandservice(roboteq_motor_controller_driver::command_srv::Request& request,     	roboteq_motor_controller_driver::command_srv::Response& response);
	bool multicommandservice(roboteq_motor_controller_driver::command_srv::Request& request,     	roboteq_motor_controller_driver::command_srv::Response& response);
	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request& request,     	roboteq_motor_controller_driver::maintenance_srv::Response& response);
	bool emergencystopservice(roboteq_motor_controller_driver::emergency_stop_srv::Request &request, roboteq_motor_controller_driver::emergency_stop_srv::Response &response);
	bool safetystopservice(roboteq_motor_controller_driver::safety_stop_srv::Request &request, roboteq_motor_controller_driver::safety_stop_srv::Response &response);
	void initialize_services();


	enum fault_flag
	{
	NO_FAULT = 0,
	OVERHEAT = 1,
	OVERVOLTAGE = 2,
	UNDERVOLTAGE = 4,
	SHORT_CIRCUIT = 8,
	EMERGENCY_STOP = 16,
	SETUP_FAULT = 32,
	MOSFET_FAILURE = 64,
	STARTUP_CONFIG_FAULT = 128,
	};

	enum status_flag
	{
	No_Fault = 0,
	Amps_Limit = 1,
	Motor_Stalled = 2,
	Loop_Error = 4,
	Safety_Stop = 8,
	Forward_Limit = 16,
	Reverse_Limit = 32,
	Amps_Trigger = 64,
	};

};
}
#endif // ROBOTEQ_MOTOR_CONTROLLER_DRIVER_MAIN_H

