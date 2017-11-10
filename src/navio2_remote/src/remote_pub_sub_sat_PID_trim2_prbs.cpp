#include "RCInput.h"
#include "PWM.h"
#include "Util.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Imu.h"
#include <sstream>

#define MOTOR_PWM_OUT 9
#define SERVO_PWM_OUT 0

//Maximum Integration angle
#define MAX_IERR 4
#define PRBS_FREQ 25
#define PI 3.14159
#define MAX_ROLL_ANGLE 20.0f

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;

float err;
float derr;
float derr_filt;
float Kierr;

//parameters to pass
float Kp;
float Ki;
float Kd;
float servo_trim;

int pid_Servo_Output(int desired_roll)
{
	//calculate errors
	float previousErr = err;
	ROS_INFO("Error %f", err);
	
	err = desired_roll - currentRoll;
	ROS_INFO("new Err %f", err);
	long timeNow = currentTime.nsec;
	ROS_INFO("Time now %d", timeNow);
	ROS_INFO("prev time %d", previousTime.nsec);
	//time between now and last roll message we got
	double dTnsec = (timeNow - previousTime.nsec);///(10e9f); //in sec
	if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
	double dT = dTnsec/(1e9f);

	ROS_INFO("Dt = %f", dT);
	if(dT > 0)
		derr = (err - previousErr)/dT;
	ROS_INFO("dErr = %f", derr);

	//filtering of derivative
	float tau = 0.025f; //matlab
	float alpha = 0.01f/(0.01f+tau);
	derr_filt = alpha*derr + (1.0f-alpha)*derr_filt;
	ROS_INFO("dErr_filt = %f", derr_filt);

	Kierr += Ki*err*dT;
	ROS_INFO("KiErr = %f", Kierr);
	//old anti wind-up (saturation)
	if(Kierr > MAX_IERR) Kierr = MAX_IERR;
	if(Kierr < -MAX_IERR) Kierr = -MAX_IERR;
	//ROS_INFO("ierr = %f", ierr);
	
	//PID CONTROLLER
	float controlSignal = Kp*err + Kierr + Kd*derr; // should be between +- 22 deg
	
	int pwmSignal = (int)(((-controlSignal*250.0f)/22.0f)+((float)servo_trim));
	if(pwmSignal > 1750) pwmSignal = 1750;
	if(pwmSignal < 1250) pwmSignal = 1250;
	ROS_INFO("control signal %f", controlSignal);
	ROS_INFO("pwm signal %d", pwmSignal);
	return pwmSignal; 
}

void read_Imu(sensor_msgs::Imu imu_msg)
{
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;
	ROS_INFO("time prev %d time now %d",previousTime.nsec, currentTime.nsec );
	//current roll angle
	currentRoll = imu_msg.orientation.x;
	ROS_INFO("current roll %f", currentRoll);
}

int main(int argc, char **argv)
{
	ROS_INFO("Start");
	int saturation = 2000;
	int freq = 100;
	Kp = 0.6;
	Ki = 2;
	Kd = 0.01;
	servo_trim = 1500;
	int prbs_val = 0;

	ROS_INFO("number of argc %d", argc);

	if(argc == 1)
	{
		//case with default params
	}
	else if(argc == 2)
	{
		//case with frequency
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	}
	else if(argc == 3)
	{
		//case with frequency and saturation
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	
		if(atoi(argv[2]) > 2000) saturation = 2000;
		else saturation = atoi(argv[2]);
	}
	else if(argc == 6)
	{
		//case with frequency and saturation
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	
		if(atoi(argv[2]) > 2000) saturation = 2000;
		else saturation = atoi(argv[2]);

		Kp = atof(argv[3]);
		Ki = atof(argv[4]);
		Kd = atof(argv[5]);
		
	}
	else if(argc == 7)
	{
		//case with frequency and saturation
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	
		if(atoi(argv[2]) > 2000) saturation = 2000;
		else saturation = atoi(argv[2]);

		Kp = atof(argv[3]);
		Ki = atof(argv[4]);
		Kd = atof(argv[5]);

		servo_trim = atoi(argv[6]);
		
	}
	else if(argc == 8)
	{
		//case with frequency and saturation
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	
		if(atoi(argv[2]) > 2000) saturation = 2000;
		else saturation = atoi(argv[2]);

		Kp = atof(argv[3]);
		Ki = atof(argv[4]);
		Kd = atof(argv[5]);

		servo_trim = atoi(argv[6]);

		prbs_val = atoi(argv[7]);
		if(prbs_val > 30 || prbs_val < 0)
		{
			ROS_INFO("prbs val must be between 0 and 500");
			return 0;
		}

		
	}
	else
	{
		ROS_INFO("not enough arguments ! Specify prbs value and throttle saturation.");
		return 0;
	}

	ROS_INFO("frequency %d, and saturation  : %d, Trim", freq, saturation, servo_trim);


 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "remote_reading_handler");
	ros::NodeHandle n;
	ros::Publisher remote_pub = n.advertise<sensor_msgs::Temperature>("remote_readings", 1000);
	ros::Publisher control_pub = n.advertise<sensor_msgs::Temperature>("control_readings", 1000);
	
	//subscribe to imu topic
	ros::Subscriber imu_sub = n.subscribe("imu_readings", 1000, read_Imu);

	//running rate = freq Hz
	ros::Rate loop_rate(freq);

	/****************************/
	/* Initialize the PID Stuff */
	/****************************/

	currentRoll = 0;
	currentTime = ros::Time::now();
	previousTime = ros::Time::now();
	Kierr = 0;
	err = 0;
	derr = 0;
	derr_filt = 0;
	
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

	sensor_msgs::Temperature rem_msg;
	sensor_msgs::Temperature ctrl_msg;

	float desired_roll = 0;

	//prbs start state
	int start_state = 0x7D0;
	int lfsr = start_state;

	int roll_low =  - prbs_val;
	int roll_high=  + prbs_val;
	int roll_prbs = roll_low;

	int ctr = 0; //counter for the period divider 

	//speed in m/s
	float speed = 0;
	float speed_filt = 0;
	int dtf = 0;// dtf read from arduino. dtf = dt*4 in msec
	float R = 0.0625f; //Rear Wheel Radius

	while (ros::ok())
	{
		
		ctr %= freq/PRBS_FREQ;


		//Throttle saturation
		if(rcin.read(3) >= saturation)
			motor_input = saturation;
		else
			motor_input = rcin.read(3);

		//read desired roll angle with remote ( 1250 to 1750 ) to range of -30 to 30 deg
		desired_roll = -((float)rcin.read(2)-1500.0f)*MAX_ROLL_ANGLE/250.0f;
		ROS_INFO("rcin usec = %d    ---   desired roll = %f", rcin.read(2), desired_roll);

		//roll PRBS
		if(!ctr)
		{
			int bit = ((lfsr >> 0) ^ (lfsr >> 2)) & 1;
			lfsr = (lfsr >> 1) | (bit << 8); //was bit << 10 before

			if (bit == 1)
				roll_prbs = roll_high;
			else if (bit == 0)
				roll_prbs = roll_low;
			else
				roll_prbs = desired_roll;
		}
		ctr++;
		if (desired_roll > roll_high || desired_roll < roll_low)
			desired_roll = desired_roll;
		else
			desired_roll = roll_prbs;

		//calculate output to servo from pid controller
		servo_input = pid_Servo_Output(desired_roll);
		
		//write readings on pwm output
		motor.set_duty_cycle(MOTOR_PWM_OUT, ((float)motor_input)/1000.0f); 
		servo.set_duty_cycle(SERVO_PWM_OUT, ((float)servo_input)/1000.0f);
		
		//save values into msg container a
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = motor_input;
		rem_msg.variance = servo_input;


		dtf = rcin.read(4)-1000;
		speed = 4.0f*PI*R*1000.0f/((float)dtf);
		if(speed < 0 || dtf < 40) speed = 0;

		
		// low pass filtering of the speed with tau = 0.1
		float alpha = (1.0f/freq)/((1.0f/freq)+0.1f);
		speed_filt = alpha*speed + (1.0f-alpha)*speed_filt;

		//save values into msg container for the control readings

		ctrl_msg.header.stamp = ros::Time::now();
		ctrl_msg.temperature = speed_filt;
		ctrl_msg.variance = desired_roll;//here it's supposed to be the desired roll


		//debug info
		printf("[Thrust:%d] - [Steering:%d] - [dtf:%4d] - [Speed:%2.2f]\n", motor_input, servo_input, dtf, speed_filt);

		//remote_pub.publish(apub);
		remote_pub.publish(rem_msg);
		control_pub.publish(ctrl_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}

