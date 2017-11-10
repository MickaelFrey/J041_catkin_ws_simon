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
#define MAX_ROLL_ANGLE 10.0f
#define SERVO_TRIM 1440.0f

// PID for roll angle
#define Kp 0.3f
#define Ki 0.0f
#define Kd 0.03f

//full range of motor
#define MAX_IERR_MOTOR 20.6

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;

float currentSpeed;
ros::Time currentTimeSpeed;
ros::Time previousTimeSpeed;

float err;
float derr;
float Kierr;

float err_m;
float derr_m;
float Kierr_m;

//parameters to pass
float Kp_m;
float Ki_m;
float Kd_m;

float RollOffset = 0; // offset to add to roll measurement for initial calibration

int the_time = 0;

int pid_Servo_Output(int desired_roll)
{//	ROS_INFO("pid servo");
	//calculate errors
	float previousErr = err;
	
	err = desired_roll - currentRoll;

	long timeNow = currentTime.nsec;

	//time between now and last roll message we got
	double dTnsec = (timeNow - previousTime.nsec);///(10e9f); //in sec
	if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
	double dT = dTnsec/(1e9f);

	if(dT > 0)
		derr = (err - previousErr)/dT;

	Kierr += Ki*err*dT;

	//old anti wind-up (saturation)
	if(Kierr > MAX_IERR) Kierr = MAX_IERR;
	if(Kierr < -MAX_IERR) Kierr = -MAX_IERR;
	
	//PID CONTROLLER
	float controlSignal = Kp*err + Kierr + Kd*derr; // should be between +- 22 deg
	
	int pwmSignal = (int)((-controlSignal*250.0f)/22.0f)+(SERVO_TRIM);
	if(pwmSignal > 1750) pwmSignal = 1750;
	if(pwmSignal < 1250) pwmSignal = 1250;

	return pwmSignal; 
}

int pid_Motor_Output(int desired_speed) // desired speed in m/s
{//	ROS_INFO("Pid motor");
	//calculate errors
	float previousErr = err_m;
	
	err_m = desired_speed - currentSpeed;

	long timeNow = currentTimeSpeed.nsec;

	//time between now and last roll message we got
	double dTnsec = (timeNow - previousTimeSpeed.nsec);///(10e9f); //in sec
	if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
	double dT = dTnsec/(1e9f);

	if(dT > 0)
		derr_m = (err_m - previousErr)/dT;

	Kierr_m += Ki_m*err_m*dT;

	//old anti wind-up (saturation)
	if(Kierr_m > MAX_IERR_MOTOR) Kierr_m = MAX_IERR_MOTOR;
	if(Kierr_m < -MAX_IERR_MOTOR) Kierr_m = -MAX_IERR_MOTOR;
	
	//PID CONTROLLER
	float controlSignal = Kp_m*err_m + Kierr_m + Kd_m*derr_m; // should be between 0 and 20.6m/s (3900*8.4*0.4*0.24*2*pi/60*62.5*10-3)
	
	int pwmSignal = (int)((controlSignal*500.0f)/20.6f)+1500;
	if(pwmSignal > 2000) pwmSignal = 2000;
	if(pwmSignal < 1500) pwmSignal = 1500;

	return pwmSignal; 
}

void read_Imu(sensor_msgs::Imu imu_msg)
{//	ROS_INFO("IMU");
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;
	//ROS_INFO("time prev %d time now %d",previousTime.nsec, currentTime.nsec );
	//current roll angle
	currentRoll = imu_msg.orientation.x;
	ROS_INFO("Time %d", the_time);
	//ROS_INFO("current roll %f", currentRoll);
	if(the_time < 15) RollOffset = currentRoll;
	//ROS_INFO("Roll Offset %f", RollOffset);
	currentRoll -= RollOffset;
	ROS_INFO("New Roll %f", currentRoll);
}

int main(int argc, char **argv)
{
//	ROS_INFO("Start");
	int saturation = 2000;
	int freq = 100;
	Kp_m = 0;
	Ki_m = 0;
	Kd_m = 0;
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

		Kp_m = atof(argv[3]);
		Ki_m = atof(argv[4]);
		Kd_m = atof(argv[5]);
		
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

		Kp_m = atof(argv[3]);
		Ki_m = atof(argv[4]);
		Kd_m = atof(argv[5]);

		prbs_val = atoi(argv[6]);
		if(prbs_val > 30 || prbs_val < 0)
		{
			ROS_INFO("prbs val must be between 0 and 30");
			return 0;
		}
		
	}
	else
	{
		ROS_INFO("not enough arguments ! Specify prbs value and throttle saturation.");
		return 0;
	}

	ROS_INFO("frequency %d, and saturation  : %d", freq, saturation);


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

	currentSpeed = 0;
	currentTimeSpeed = ros::Time::now();
	previousTimeSpeed = ros::Time::now();
	Kierr_m = 0;
	err_m = 0;
	derr_m = 0;
	
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
	//ROS_INFO("l");
	motor.enable(MOTOR_PWM_OUT);
	servo.enable(SERVO_PWM_OUT);

	motor.set_period(MOTOR_PWM_OUT, 50); //frequency 50Hz
	servo.set_period(SERVO_PWM_OUT, 50); 

	int motor_input = 0;
	int servo_input = 0;

	sensor_msgs::Temperature rem_msg;
	sensor_msgs::Temperature ctrl_msg;

	float desired_roll = 0;
	float desired_speed = 0;

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
	//ROS_INFO("Init");
	RollOffset = 0;
	int initTime = ros::Time::now().sec%1000;
	while (ros::ok())
	{
	//	ROS_INFO("Loop");
		ctr %= freq/PRBS_FREQ;

		//read desired roll angle with remote ( 1250 to 1750 ) to range of -30 to 30 deg
		desired_roll = -((float)rcin.read(2)-1500.0f)*MAX_ROLL_ANGLE/250.0f;
		//ROS_INFO("rcin usec = %d    ---   desired roll = %f", rcin.read(2), desired_roll);

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
		if (desired_roll > roll_high || desired_roll < roll_low ||
			 currentRoll > MAX_ROLL_ANGLE ||
			 currentRoll < -MAX_ROLL_ANGLE)
			desired_roll = desired_roll;
		else
			desired_roll = roll_prbs;


		//Get Desired PWM Speed using Throttle saturation
		int desired_pwm = 0;
		if(rcin.read(3) >= saturation)
			desired_pwm = saturation;
		else
			desired_pwm = rcin.read(3);
	//	ROS_INFO("mid");
		//get derired speed in m/s using desired pwm
		desired_speed = 20.6f*((float)desired_pwm-1500)/(500.0f);
		if(desired_speed < 0) desired_speed = 0.0f;

		//Read current Speed in m/s
		dtf = rcin.read(5)-1000;
		speed = 4.0f*PI*R*1000.0f/((float)dtf);
		if(speed < 0 || dtf < 40) speed = 0;
		
		// low pass filtering of the speed with tau = 0.1
		float alpha = (1.0f/freq)/((1.0f/freq)+0.1f);
		speed_filt = alpha*speed + (1.0f-alpha)*speed_filt;

		//update time for speed control
		currentSpeed = speed_filt;
		previousTimeSpeed = currentTimeSpeed;
		currentTimeSpeed = ros::Time::now();

		//calculate output to motor from pid controller
		motor_input = pid_Motor_Output(desired_speed);
		if(desired_pwm < 1500)
			motor_input = desired_pwm;
		//calculate output to servo from pid controller
		servo_input = pid_Servo_Output(desired_roll);
		
		//write readings on pwm output
		motor.set_duty_cycle(MOTOR_PWM_OUT, ((float)motor_input)/1000.0f); 
		servo.set_duty_cycle(SERVO_PWM_OUT, ((float)servo_input)/1000.0f);

		
		//ROS_INFO("Ros Time");
		the_time = ros::Time::now().sec%1000-initTime;
		//ROS_INFO("%d", time);
		//Calibration of Roll measurement !
		


		//save values into msg container a
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = desired_speed;//motor_input;
		rem_msg.variance = desired_roll;//servo_input;

		//save values into msg container for the control readings
		ctrl_msg.header.stamp = ros::Time::now();
		ctrl_msg.temperature = currentSpeed;
		ctrl_msg.variance = currentRoll;//desired_roll;//here it's supposed to be the desired roll

		ROS_INFO("DESIRED SPEED : %2.2f     CURRENT SPEED %2.2f", desired_speed, currentSpeed);
		//debug info
		//printf("[Thrust:%d] - [Steering:%d] - [dtf:%4d] - [Speed:%2.2f]\n", motor_input, servo_input, dtf, speed_filt);
		printf("RcVals > %4d %4d %4d %4d %4d %4d %4d %4d\n",rcin.read(0), rcin.read(1), rcin.read(2), rcin.read(3), rcin.read(4), rcin.read(5), rcin.read(6), rcin.read(7));
		//remote_pub.publish(apub);
		remote_pub.publish(rem_msg);
		control_pub.publish(ctrl_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}

