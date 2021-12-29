#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>

class Odometry_calc{

public:
	Odometry_calc();

	void spin();

private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber l_wheel_sub;
	ros::Subscriber r_wheel_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_lencoder;
	double prev_rencoder;

	double lmult;
	double rmult;

	double left;
	double right;

	double rate;

	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;


	double enc_left ;

	double enc_right;

	double ticks_meter;

	double base_width;

	double dx;

	double dr;
	int c = 0;
	bool get_odom;
	double x_final,y_final, theta_final;

	ros::Time current_time, last_time;

	void leftencoderCb(const roboteq_motor_controller_driver::channel_values& left_ticks);
	void rightencoderCb(const roboteq_motor_controller_driver::channel_values& right_ticks);
	
	void init_variables();

	void update();
};

Odometry_calc::Odometry_calc(){

	init_variables();

	ROS_INFO("Started odometry computing node");

	l_wheel_sub = n.subscribe("/roboteq_motor_controller_driver/encoder_count",1000, &Odometry_calc::leftencoderCb, this);
	
	r_wheel_sub = n.subscribe("/roboteq_motor_controller_driver/encoder_count",1000, &Odometry_calc::rightencoderCb, this);

  	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);   
  	
	//Retrieving parameters of this node
	
}

void Odometry_calc::init_variables()
{
	prev_lencoder = 0;
	prev_rencoder = 0;

	lmult = 0;
	rmult = 0;

	left = 0;
	right = 0;

	encoder_min =  -65536;
	encoder_max =  65536;

	rate = 5;

	ticks_meter = (1024 * 24.69 * 4) / (2 * M_PI * 0.105); // 1024=PPR 24.69=Gear Ratio 4=CPR/PPR 0.105=Radius of wheel  

	base_width = 1; // distance between wheels

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;
	
	then = ros::Time::now();
	x_final = 0 ; y_final = 0; theta_final = 0;
	enc_left = left;
	enc_right =right;

	dx = 0;
	dr = 0;
	
	get_odom = false;

 	current_time = ros::Time::now();
  	last_time = ros::Time::now();

}

//Spin function
void Odometry_calc::spin(){

     ros::Rate loop_rate(rate);

     while (ros::ok())
	{	
		
		update();
		loop_rate.sleep();
	}

}

//Update function
void Odometry_calc::update(){

	ros::Time now = ros::Time::now();
	
//	ros::Time elapsed;

	double elapsed;

	double d_left, d_right, d, th,x,y;
	
	if ( now > t_next) {

		elapsed = now.toSec() - then.toSec(); 

 // 	ROS_INFO_STREAM("elapsed =" << elapsed);

		if(get_odom == true){
		if(left == 0 && right == 0) {
			d_left = 0;
			d_right = 0;
		

		
		}
		else{
			d_left = (left - enc_left) / ( ticks_meter);
			d_right = (right - enc_right) / ( ticks_meter);
		}
		
		enc_left = left;
		ROS_INFO_STREAM("left __="<<left);
		enc_right = right;
		ROS_INFO_STREAM("right__ ="<<right);
		
		d = (d_left + d_right ) / 2.0;
		//ROS_INFO_STREAM("d =" << d);
		//ROS_INFO_STREAM(d_left << " : " << d_right);
		th = ( d_right - d_left ) / base_width;
		//ROS_INFO_STREAM("th =" <<th);
		dx = d /elapsed;
		//ROS_INFO_STREAM("dx =" << dx);
		dr = th / elapsed;
		//ROS_INFO_STREAM("theta_final =" << theta_final);
		//ROS_INFO_STREAM("x_final =" << x_final);
		//ROS_INFO_STREAM("Y_final =" << y_final);
	
		if ( d != 0){

                	x = cos( th ) * d;
                	//ROS_INFO_STREAM(x);
                	y = sin( th ) * d;
                	// calculate the final position of the robot
                	x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
                	y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
					//x_final = x_final + x;
					//y_final = y_final + y;
			}
	

           	 if( th != 0){
                	theta_final = theta_final + th;
				}
		    geometry_msgs::Quaternion odom_quat ;
			//ROS_INFO_STREAM("theta_final =" << theta_final);
			//ROS_INFO_STREAM("x_final =" << x_final);
			//ROS_INFO_STREAM("Y_final =" << y_final);
		    odom_quat.x = 0.0;
		    odom_quat.y = 0.0;
		    odom_quat.z = sin( theta_final / 2 );	
            odom_quat.w = cos( theta_final / 2 );

		    //first, we'll publish the transform over tf
		    geometry_msgs::TransformStamped odom_trans;
		    odom_trans.header.stamp = now;
		    odom_trans.header.frame_id = "odom";
		    odom_trans.child_frame_id = "base_link";

		    odom_trans.transform.translation.x = x_final;
		    odom_trans.transform.translation.y = y_final;
		    odom_trans.transform.translation.z = 0.0;  // 2D Z axis is 0
		    odom_trans.transform.rotation = odom_quat;

		    //send the transform
		    odom_broadcaster.sendTransform(odom_trans);

		    //next, we'll publish the odometry message over ROS
		    nav_msgs::Odometry odom;
		    odom.header.stamp = now;
		    odom.header.frame_id = "odom";

		    //set the position
		    odom.pose.pose.position.x = x_final;
		    odom.pose.pose.position.y = y_final;
		    odom.pose.pose.position.z = 0.0;
		    odom.pose.pose.orientation = odom_quat;

		    //set the velocity
		    odom.child_frame_id = "base_link";
		    odom.twist.twist.linear.x = dx;
		    odom.twist.twist.linear.y = 0;
		    odom.twist.twist.angular.z = dr;

		    //publish the message
		    odom_pub.publish(odom);
		}
	    	    then = now;

//		    ROS_INFO_STREAM("dx =" << x_final);

//		    ROS_INFO_STREAM("dy =" << y_final);

	            ros::spinOnce();


		}
	 else { ; }
//		ROS_INFO_STREAM("Not in loop");
		
		



}

void Odometry_calc::leftencoderCb(const roboteq_motor_controller_driver::channel_values& left_ticks)
{

// ROS_INFO_STREAM("Left tick" << left_ticks->data);
	double enc = left_ticks.value[0];
	//ROS_INFO_STREAM("enc =" << enc);
	
	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		lmult = lmult + 1;
		//ROS_INFO_STREAM("LMULT:" << lmult);
	}
	
	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))
	{
		lmult = lmult - 1;
		//ROS_INFO_STREAM("LMULT:" << lmult);
	}
	
	left = 1.0 * (enc + lmult * (encoder_max - encoder_min));

	prev_lencoder = enc;
 	//ROS_INFO_STREAM("Left " << left);
}

void Odometry_calc::rightencoderCb(const roboteq_motor_controller_driver::channel_values& right_ticks)
{	
	double enc = -right_ticks.value[1];
	ROS_INFO_STREAM("Right_enc " << enc);
	get_odom = true;
	if((enc < encoder_low_wrap) && (prev_rencoder > encoder_high_wrap))
	{
		rmult = rmult + 1;
		//ROS_INFO_STREAM("LMULT:" << lmult);
	}
	
	if((enc > encoder_high_wrap) && (prev_rencoder < encoder_low_wrap))
	{
		rmult = rmult - 1;
		//ROS_INFO_STREAM("LMULT:" << lmult);
	}
	
	right = 1.0 * (enc + rmult * (encoder_max - encoder_min));

	prev_rencoder = enc;
	//ROS_INFO_STREAM("Right " << right);

}







int main(int argc, char **argv)

{
	ros::init(argc, argv,"diff_odom");
	Odometry_calc obj;
	obj.spin();


	return 0;

}
