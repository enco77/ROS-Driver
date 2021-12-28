#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>
class RoboteqDriver
{
public:
	RoboteqDriver()
	{
		initialize(); //constructor - Initialize
	}

	~RoboteqDriver()
	{
		if (ser.isOpen())
		{
			ser.close();
		}
	}

	serial::Serial ser;
	std::string port;
	int32_t baud;
	ros::Publisher read_publisher;
	ros::Subscriber cmd_vel_sub;

	int frequency;
	float wheelbase;
	float radius;
	float gearRatio;
	float maxRPM;
	ros::NodeHandle nh = ros::NodeHandle("~");

	void initialize()
	{

		nh.getParam("port", port);
		nh.getParam("baud", baud);
		nh.getParam("wheelbase", wheelbase);
		nh.getParam("radius", radius);
		nh.getParam("gear_ratio", gearRatio);
		nh.getParam("max_rpm", maxRPM);
		cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &RoboteqDriver::cmd_vel_callback, this);
		connect();
	}

	void connect()
	{
		try
		{
			ser.setPort(port);
			ser.setBaudrate(baud); //get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{

			ROS_ERROR_STREAM("Unable to open port ");
			ROS_INFO_STREAM("Unable to open port");
		}
		if (ser.isOpen())
		{
			ROS_INFO_STREAM("Serial Port initialized\"");
		}
		else
		{
			ROS_INFO_STREAM("Serial Port is not open");
		}
		run();
	}

	void cmd_vel_callback(const geometry_msgs::Twist &msg)
	{
		std::stringstream mcommands;
		mcommands << "!S 1"
				<< " " << to_rpm(calculate_right_speed(msg.linear.x, msg.angular.z)) << "_"
				<< "!S 2"
				<< " " << to_rpm(calculate_left_speed(msg.linear.x, msg.angular.z)) << "_";
		ser.write(mcommands.str());
		ser.flush();
		// ROS_INFO_STREAM(mcommands.str());
	}

	double calculate_right_speed(double x, double z)
	{
		return (2 * x + z * wheelbase) / (2 * radius);
	}

	double calculate_left_speed(double x, double z)
	{
		return (z * wheelbase - 2 * x ) / (2 * radius);
	}

	double to_rpm(double value)
	{

		return max_limit((value * 60 * gearRatio) / (2 * M_PI));
	}

	double max_limit(double speed)
	{
		if (speed > 0)
		{
			return std::min<double>(maxRPM, speed);
		}
		else return std::max<double>(-maxRPM, speed);
	}

	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer multicommandsrv;
	ros::ServiceServer maintenancesrv;
	ros::ServiceServer emergencysrv;
	ros::ServiceServer safetystopsrv;


	bool configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
			<< "%\clsav321654987";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		// ROS_INFO_STREAM(response.result);
		return true;
	}

	bool commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		// ROS_INFO_STREAM(response.result);
		return true;
	}

	bool multicommandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "!" << request.userInput << " " << "1" << " " << request.value << "_" << "!" << request.userInput << " " << "2" << " " << request.value << "_";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		// ROS_INFO_STREAM(response.result);
		return true;
	}

	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "%" << request.userInput << " "
			<< "_";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		// ROS_INFO_STREAM(response.result);
		return true;
	}

	bool emergencystopservice(roboteq_motor_controller_driver::emergency_stop_srv::Request &request, roboteq_motor_controller_driver::emergency_stop_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		if (request.state)
		{
			str << "!" << "EX" << "_";
		}
		else
		{
			str << "!" << "MG" << "_";
		}
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		// ROS_INFO_STREAM(response.result);
		return true;
	}

	bool safetystopservice(roboteq_motor_controller_driver::safety_stop_srv::Request &request, roboteq_motor_controller_driver::safety_stop_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		if (request.state)
		{
			str << "!" << "SFT 1" << "_" << "!" << "SFT 2" << "_";
		}
		else
		{
			for (int i=0; i<10; i++)
			{
				str << "!S 1 0" << "_" << "!S 2 0" << "_";
				sleep(0.1);
			}
		}
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		// ROS_INFO_STREAM(response.result);
		return true;
	}


	void initialize_services()
	{
		configsrv = nh.advertiseService("config_service", &RoboteqDriver::configservice, this);
		commandsrv = nh.advertiseService("command_service", &RoboteqDriver::commandservice, this);
		multicommandsrv = nh.advertiseService("dualchannel_command_service", &RoboteqDriver::multicommandservice, this);
		maintenancesrv = nh.advertiseService("maintenance_service", &RoboteqDriver::maintenanceservice, this);
		emergencysrv = nh.advertiseService("emergency_stop_service", &RoboteqDriver::emergencystopservice, this);
		safetystopsrv = nh.advertiseService("safety_stop_service", &RoboteqDriver::safetystopservice, this);
	}

	void run()
	{
		initialize_services();
		std_msgs::String str1;
		nh.getParam("frequency", frequency);
		typedef std::string Key;
		typedef std::string Val;
		std::map<Key, Val> map_sH;
		nh.getParam("query", map_sH);

		std::stringstream ss0;
		std::stringstream ss1;
		std::stringstream ss2;
		std::stringstream ss3;
		std::vector<std::string> KH_vector;

		ss0 << "^echof 1_";
		ss1 << "# c_/\"DH?\",\"?\"";
		for (std::map<Key, Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter)
		{
			Key KH = iter->first;

			KH_vector.push_back(KH);

			Val VH = iter->second;

			ss1 << VH << "_";
		}
		ss1 << "# " << frequency << "_";
		// ROS_INFO_STREAM(ss1.str());
		std::vector<ros::Publisher> publisherVecH;
		for (int i = 0; i < KH_vector.size(); i++)
		{
			publisherVecH.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KH_vector[i], 100));
		}
		ser.write(ss0.str());
		ser.write(ss1.str());
		ser.write(ss2.str());
		ser.write(ss3.str());

		ser.flush();
		int count = 0;
		read_publisher = nh.advertise<std_msgs::String>("read", 1000);
		sleep(2);
		ros::Rate loop_rate(5);
		while (ros::ok())
		{

			ros::spinOnce();
			if (ser.available())
			{

				std_msgs::String result;
				result.data = ser.read(ser.available());

				read_publisher.publish(result);
				boost::replace_all(result.data, "\r", "");
				boost::replace_all(result.data, "+", "");

				std::vector<std::string> fields;

				std::vector<std::string> Field9;
				boost::split(fields, result.data, boost::algorithm::is_any_of("D"));

				std::vector<std::string> fields_H;
				boost::split(fields_H, fields[1], boost::algorithm::is_any_of("?"));

				if (fields_H[0] == "H")
				{

					for (int i = 0; i < publisherVecH.size(); ++i)
					{

						std::vector<std::string> sub_fields_H;

						boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
						roboteq_motor_controller_driver::channel_values Q1;

						for (int j = 0; j < sub_fields_H.size(); j++)
						{

							try
							{
								Q1.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
							}
							catch (const std::exception &e)
							{
								count++;
								if (count > 10)
								{
									ROS_INFO_STREAM("Garbage data on Serial");
								}
							}
						}

						publisherVecH[i].publish(Q1);
					}
				}
			}
			loop_rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");
	RoboteqDriver driver;
	ros::waitForShutdown();
	return 0;
}
