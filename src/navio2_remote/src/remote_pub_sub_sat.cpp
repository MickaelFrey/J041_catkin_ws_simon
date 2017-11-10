#include "RCInput.h"
#include "PWM.h"
#include "Util.h"
#include <unistd.h>

#include "ros/ros.h"
//#include "std_msgs/Float64MultiArray.h"
//#include "std_msgs/MultiArrayLayout.h"
//#include "std_msgs/MultiArrayDimension.h"
#include "sensor_msgs/Temperature.h"
//#include "std_msgs/Float64.h"
#include <sstream>

#define MOTOR_PWM_OUT 9
#define SERVO_PWM_OUT 0

int main(int argc, char **argv)
{
	ROS_INFO("Start");
	int saturation = 2000;

	ROS_INFO("argc %d", argc);
	if(argc > 1)
	{
		//case where there is a saturation request
		if(atoi(argv[1]) > 2000) saturation = 2000;
		else saturation = atoi(argv[1]);
		ROS_INFO("saturation  : %d", saturation);
	}

 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "remote_reading_handler");
	ros::NodeHandle n;
	//ros::Publisher remote_pub = n.advertise<std_msgs::Float64MultiArray>("remote_readings", 1000);
	ros::Publisher remote_pub = n.advertise<sensor_msgs::Temperature>("remote_readings", 1000);
	
	//running rate = 30 Hz
	ros::Rate loop_rate(30);

	/*******************************************/
	/* Initialize the RC input, and PWM output */
	/*******************************************/

	RCInput rcin;
	rcin.init();
	PWM servo;
	PWM motor;

	if (!motor.init(MOTOR_PWM_OUT)) {
		fprintf(stderr, "Motor Output Enable not set. Are you root?\n");
		return 0;
    	}

	if (!servo.init(SERVO_PWM_OUT)) {
		fprintf(stderr, "Servo Output Enable not set. Are you root?\n");
		return 0;
    	}

	motor.enable(MOTOR_PWM_OUT);
	servo.enable(SERVO_PWM_OUT);

	motor.set_period(MOTOR_PWM_OUT, 50); //frequency 50Hz
	servo.set_period(SERVO_PWM_OUT, 50); 

	int motor_input = 0;
	int servo_input = 0;

	//msg stuff
	/*
	std_msgs::Float64MultiArray apub;
	apub.data.push_back((double)0.0);
	apub.data.push_back((double)0.0);
	apub.layout.dim.push_back(std_msgs::MultiArrayDimension());
	apub.layout.dim[0].size = 2;
	apub.layout.dim[0].label = "REMOTEmsg";
	apub.layout.dim[0].stride = 2;
	apub.layout.data_offset = 0;
	*/
	sensor_msgs::Temperature rem_msg;

	int rpm_pwm = 0;
	int rpm = 0;

	while (ros::ok())
	{

		//read input from rc
		if(rcin.read(3) >= saturation)
			motor_input = saturation;
		else
			motor_input = rcin.read(3);

		servo_input = rcin.read(2); // Roll steering
		
		//motor_input = rcin.read(3); // Throttle

		//write readings on pwm output
		motor.set_duty_cycle(MOTOR_PWM_OUT, ((float)motor_input)/1000.0f); 
		servo.set_duty_cycle(SERVO_PWM_OUT, ((float)servo_input)/1000.0f);
		
		//save values into msg container a
		//apub.data[0] = motor_input;
		//apub.data[1] = servo_input;
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = motor_input;
		rem_msg.variance = servo_input;

		//debug info
		ROS_INFO("Thrust usec = %d    ---   Steering usec = %d", motor_input, servo_input);

		//rpm info
		rpm_pwm = rcin.read(8);
		rpm = (rpm_pwm-1000)*3;
		//ROS_INFO("rpm pwm %d , rpm = %d", rcin.read(8), (rcin.read(8)-1000)*3);
		ROS_INFO("%d %d %d %d %d %d", rcin.read(1), rcin.read(4),
			 rcin.read(5), rcin.read(6), rcin.read(7), rcin.read(8));

	

		//remote_pub.publish(apub);
		remote_pub.publish(rem_msg);
		
		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}

