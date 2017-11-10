	#include "RCInput.h"
	#include "PWM.h"
	#include "Util.h"
	#include <unistd.h>

	#include "ros/ros.h"
	#include "sensor_msgs/Temperature.h"
	#include "sensor_msgs/Imu.h"
	#include "sensor_msgs/NavSatFix.h"
	#include <sstream>

	//PWM Pins on Navio2
	#define MOTOR_PWM_OUT 9
	#define SERVO_PWM_OUT 0

	//Maximum Integration angle
	#define MAX_IERR1 4
	#define MAX_IERR2 5
	#define PI 3.14159
	#define SERVO_TRIM 1440.0f

	// PID for roll angle
	#define Kp1 0.3f
	#define Ki1 0.0f
	#define Kd1 0.03f

	// PID for roll angle outer loop
	float Kp2[3] = {0, 1.05f, 1.22f};
	float Ki2[3] = {2.23f, 5.89f, 5.83f};
	float Kd2[3] = {0, 0.0465f, 0.0195f};

	//full range of motor
	#define MAX_IERR_MOTOR 20.6

	float max_roll_angle = 30.0f;

	float currentRoll;
	ros::Time currentTime;
	ros::Time previousTime;

	float currentYaw;
	float recYaw; //Yaw information recieved

	float currentSpeed;
	ros::Time currentTimeSpeed;
	ros::Time previousTimeSpeed;

	//Variables for GPS

	float GPS_lat;
	float GPS_lon;
	double currentTimeGPS;
	double previousTimeGPS;
	double dtGPS; 
	float base_lat; //= 46.51849177;
	float base_lon; // = 6.56666458;
	int GPS_data_rec = 0; 
	int Update_phase = 0;
	int first_gps = 1;

	//Variables for Kalman
	float Kalman_P[2][2] = {{0.0, 0.0},{0.0, 0.0}};
	float Kalman_Q[2][2] = {{0.5*1/1e5, 0.0},{0.0, 0.5*1/1e5}};
	float Kalman_R[2][2] = {{0.1, 0.0},{0.0, 0.1}};
	float Kalman_S[2][2] = {{0.0, 0.0},{0.0, 0.0}};
	float Kalman_S_inv[2][2] = {{0.0, 0.0},{0.0, 0.0}};
	float Kalman_K[2][2] = {{0.0, 0.0},{0.0, 0.0}};
	float Kalman_eye[2][2] = {{1.0, 0.0},{0.0, 1.0}};
	float Kalman_eye_min_K[2][2] = {{0.0, 0.0},{0.0, 0.0}};
	float Kalman_K_ybar[2][1] = {{0.0},{0.0}};

	// note that Kalman_H is identity matrix
	// note that the jacobian of the system is the identity matrix

	float mu_kalman[2][1] = {{0.0},{0.0}};
	float P_kk_1[2][2];
	float mu_kk_1[2][1];
	float ybar[2][1];
	float z_gps[2][1] = {{0.0},{0.0}};

	//Roll Errors 1
	float err1;
	float derr1;
	float Kierr1;

	//Roll Errors 2
	float err2;
	float derr2;
	float Kierr2;

	//Motor Errors
	float err_m;
	float derr_m;
	float Kierr_m;

	//Motor PID parameters to pass
	float Kp_m;
	float Ki_m;
	float Kd_m;

	float RollOffset = 0; // offset to add to roll measurement for initial calibration
	float YawOffset = 0; //Due to the fact that the yaw starts at the same value regardeless of real yaw angle

	int the_time = 0;

	//this function outputs the outer loop controller roll reference angle
	float pid_Ref_Output(int desired_roll) //in degrees
	{
		int idx = 0;
		if(currentSpeed < 4.5f) {idx = 0; max_roll_angle = 30.0f;}
		else if(currentSpeed < 5.5f && currentSpeed >= 4.5f) {idx = 1; max_roll_angle = 30.0f;}
		else {idx = 2; max_roll_angle = 30.0f;}
		
		//calculate errors
		float previousErr = err2;
		err2 = desired_roll - currentRoll;

		/*
		long timeNow = currentTime.nsec;
		//time between now and last roll message we got
		double dTnsec = (timeNow - previousTime.nsec); // in nanoseconds
		if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
		double dT = dTnsec/(1e9f);*/

		double dT = currentTime.toSec()-previousTime.toSec();
		//printf("dtyaw%f\n", dT);

		if(dT > 0)
			derr2 = (err2 - previousErr)/dT;

		Kierr2 += Ki2[idx]*err2*dT;

		//anti wind-up (saturation)
		if(Kierr2 > MAX_IERR2) Kierr2 = MAX_IERR2;
		if(Kierr2 < -MAX_IERR2) Kierr2 = -MAX_IERR2;
		
		//PID CONTROLLER
		float controlSignal = Kp2[idx]*err2 + Kierr2 + Kd2[idx]*derr2; // should be between +- 30 deg (roll limit)
		if(controlSignal > max_roll_angle) controlSignal = max_roll_angle;
		if(controlSignal < -max_roll_angle) controlSignal = -max_roll_angle;

		return controlSignal; 
	}

	int pid_Servo_Output(int desired_roll) //in degrees
	{
		//calculate errors
		float previousErr = err1;
		err1 = desired_roll - currentRoll;

		/*long timeNow = currentTime.nsec;

		//time between now and last roll message we got
		double dTnsec = (timeNow - previousTime.nsec); // in nanoseconds
		if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
		double dT = dTnsec/(1e9f);
		*/
		double dT = currentTime.toSec()-previousTime.toSec();
		//printf("dtyaw ds output %f\n", dT);

		if(dT > 0)
			derr1 = (err1 - previousErr)/dT;

		Kierr1 += Ki1*err1*dT;

		//anti wind-up (saturation)
		if(Kierr1 > MAX_IERR1) Kierr1 = MAX_IERR1;
		if(Kierr1 < -MAX_IERR1) Kierr1 = -MAX_IERR1;
		
		//PID CONTROLLER
		float controlSignal = Kp1*err1 + Kierr1 + Kd1*derr1; // should be between +- 22 deg (steer limit)
		
		int pwmSignal = (int)((-controlSignal*250.0f)/22.0f)+(SERVO_TRIM);
		if(pwmSignal > 1750) pwmSignal = 1750;
		if(pwmSignal < 1250) pwmSignal = 1250;

		return pwmSignal; 
	}

	//never called function
	int pid_Motor_Output(int desired_speed) // desired speed in m/s
	{
		//calculate errors
		float previousErr = err_m;
		err_m = desired_speed - currentSpeed;

		long timeNow = currentTimeSpeed.nsec;

		//time between now and last roll message we got
		double dTnsec = (timeNow - previousTimeSpeed.nsec); // in nanoseconds
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
	{
		//save the time of the aquisition
		previousTime = currentTime;
		currentTime = imu_msg.header.stamp;

		//current yaw angle (for GPS kalman filtering)
		recYaw = imu_msg.orientation.z;
		//current roll angle
		currentRoll = imu_msg.orientation.x;
		//ROS_INFO("Time %d", the_time);

		//keep calibration after 15 seconds
		if(the_time < 15) RollOffset = currentRoll;
		if(the_time < 20) YawOffset = 180 - recYaw; //Initialize motorbike with an orientation of pi (west direction)

		recYaw += YawOffset;
		currentYaw = recYaw*3.141592/180.0; //Converting into radians 
		currentRoll -= RollOffset;
		//ROS_INFO("New Roll %f", currentRoll);
	}

	void read_GPS(sensor_msgs::NavSatFix gps_msg)
	{
		//save the time of the aquisition
		previousTimeGPS = currentTimeGPS;
		currentTimeGPS = gps_msg.header.stamp.toSec();

		//current lat lon and dt
		GPS_lat = gps_msg.latitude;
		GPS_lon = gps_msg.longitude;
		dtGPS = (currentTimeGPS - previousTimeGPS);
		GPS_data_rec += 1;

		//ROS_INFO("dt: %f - Lat: %f - Lon: %f", dtGPS, GPSLat, GPSLon);
	}

	float Kalman_evalX (float x, float v, float alpha, float dt){
		float x2 = x + v*cos(alpha)*dt;
		return x2;
	}

	float Kalman_evalY (float y, float v, float alpha, float dt){
		float y2 = y + v*sin(alpha)*dt;
		return y2;
	}

	void sum22 (float a[2][2], float b[2][2], float c[2][2])
	{
		c[0][0] = a[0][0] + b[0][0];
		c[0][1] = a[0][1] + b[0][1];
		c[1][0] = a[1][0] + b[1][0];
		c[1][1] = a[1][1] + b[1][1];
	}

	void substr22 (float a[2][2], float b[2][2], float c[2][2])
	{
		c[0][0] = a[0][0] - b[0][0];
		c[0][1] = a[0][1] - b[0][1];
		c[1][0] = a[1][0] - b[1][0];
		c[1][1] = a[1][1] - b[1][1];
	}

	void sum21 (float a[2][1], float b[2][1], float c[2][1])
	{
		c[0][0] = a[0][0] + b[0][0];
		c[1][0] = a[1][0] + b[1][0];
	}

	void substr21 (float a[2][1], float b[2][1], float c[2][1])
	{
		c[0][0] = a[0][0] - b[0][0];
		c[1][0] = a[1][0] - b[1][0];
	}

	void invert22 (float a[2][2], float b[2][2])
	{
		float det = a[0][0]*a[1][1] - a[0][1]*a[1][0];
		b[0][0] = 1.0/det*a[1][1];
		b[0][1] = -1.0/det*a[0][1];
		b[1][0] = -1.0/det*a[1][0];
		b[1][1] = 1.0/det*a[0][0];
	}

	void multip22by22 (float a[2][2], float b[2][2], float c[2][2])
	{
		c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
		c[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1];
		c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
		c[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1];
	}

	void multip22by21 (float a[2][2], float b[2][1], float c[2][1])
	{

		c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
		c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
	}

	void equal21 (float a[2][1], float b[2][1])
	{

		b[0][0] = a[0][0];
		b[1][0] = a[1][0];
	}

	void equal22 (float a[2][2], float b[2][2])
	{

		b[0][0] = a[0][0];
		b[1][0] = a[1][0];
		b[0][1] = a[0][1];
		b[1][1] = a[1][1];
	}

	int main(int argc, char **argv)
	{

		int saturation = 2000;
		int freq = 100;
		Kp_m = 0;
		Ki_m = 0;
		Kd_m = 0;

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
		else if(argc == 8)
		{
			//case with frequency and saturation and PID for motor
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
			base_lat = atof(argv[6]);
			base_lon = atof(argv[7]);
			
		}
		else
		{
			ROS_INFO("not enough arguments ! Specify throttle saturation.");
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
		ros::Publisher position_pub = n.advertise<sensor_msgs::Imu>("pos_readings", 1000);
		
		//subscribe to imu topic
		ros::Subscriber imu_sub = n.subscribe("imu_readings", 1000, read_Imu);
		ros::Subscriber gps_sub = n.subscribe("gps_readings", 1000, read_GPS);

		//running rate = freq Hz
		ros::Rate loop_rate(freq);

		/****************************/
		/* Initialize the PID Stuff */
		/****************************/
		
		//Roll Control
		currentRoll = 0;
		currentTime = ros::Time::now();
		previousTime = ros::Time::now();
		Kierr1 = 0;
		err1 = 0;
		derr1 = 0;
		Kierr2 = 0;
		err2 = 0;
		derr2 = 0;

		//Motor Control
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

		motor.enable(MOTOR_PWM_OUT);
		servo.enable(SERVO_PWM_OUT);

		motor.set_period(MOTOR_PWM_OUT, 50); //frequency 50Hz for PWM
		servo.set_period(SERVO_PWM_OUT, 50); 

		int motor_input = 0;
		int servo_input = 0;

		sensor_msgs::Temperature rem_msg; //use of Temperature type messages. Because 2 floats
		sensor_msgs::Temperature ctrl_msg;
		sensor_msgs::Imu pos_msg; //use of imu type message for storing position in XY and kalman filtered

		float desired_roll = 0;
		float desired_speed = 0;

		//speed in m/s
		float speed = 0;
		float speed_filt = 0;
		int dtf = 0;// dtf read from arduino. dtf = dt*4 in msec
		float R = 0.0625f; //Rear Wheel Radius

		RollOffset = 0;
		int initTime = ros::Time::now().sec%1000;


		/*******************************************/
		/*             MAIN ROS LOOP               */
		/*******************************************/

		while (ros::ok())
		{

			/*******************************************/
			/*             ROLL SECTION                */
			/*******************************************/

			//read desired roll angle with remote ( 1250 to 1750 ) to range limited by defines
			desired_roll = -((float)rcin.read(2)-1500.0f)*max_roll_angle/250.0f;
			//printf("recieved pwm %f\n", (float)rcin.read(2));
			/*******************************************/
			/*             VELOCITY SECTION            */
			/*******************************************/

			//Get Desired PWM Speed using Throttle saturation
			int desired_pwm = 0;
			if(rcin.read(3) > 1500) desired_pwm = ((float)rcin.read(3)-1500.0f)*((float)saturation - 1500.0f)/500.0f + 1500.0f;		
			else desired_pwm = rcin.read(3);

	//if(rcin.read(3) >= saturation)
			//	desired_pwm = saturation;
			//else
			//	desired_pwm = rcin.read(3);

			//get derired speed in m/s using desired pwm
			desired_speed = 20.6f*((float)desired_pwm-1500)/(500.0f);
			if(desired_speed < 0) desired_speed = 0.0f;

			//Read current Speed in m/s
			dtf = rcin.read(5)-1000;
			speed = 4.0f*PI*R*1000.0f/((float)dtf);
			if(speed < 0 || dtf < 40) speed = 0;
			
			// low pass filtering of the speed with tau = 0.1
			float alpha = (1.0f/freq)/((1.0f/freq)+0.4f);
			speed_filt = alpha*speed + (1.0f-alpha)*speed_filt;

			//update time for speed control
			currentSpeed = speed_filt;
			previousTimeSpeed = currentTimeSpeed;
			currentTimeSpeed = ros::Time::now();

			//calculate output to motor from pid controller
			motor_input = desired_pwm; // pid_Motor_Output(desired_speed);
			if(desired_pwm < 1500)
				motor_input = desired_pwm;

			//calculate output to servo from pid controller
			servo_input = pid_Servo_Output(pid_Ref_Output(desired_roll));
			
			//write readings on pwm output
			motor.set_duty_cycle(MOTOR_PWM_OUT, ((float)motor_input)/1000.0f); 
			servo.set_duty_cycle(SERVO_PWM_OUT, ((float)servo_input)/1000.0f);

			//Measure time for initial roll calibration
			the_time = ros::Time::now().sec%1000-initTime;



			/*******************************************/
			/*        KALMAN FILTERING SECTION         */
			/*******************************************/

			z_gps[0][0] = (GPS_lon - base_lon)*767.4/10000*1e6;
			z_gps[1][0] = (GPS_lat - base_lat)*1111.6/10000*1e6; //neglect the curvature of earth
			
			
			if (the_time<=20) printf("the time : %d - Calibration (Roll = 0 , Yaw = 180) \n" , the_time);
			if (the_time>20)
			{

				if (first_gps == 0) //initialize the first value of GPS to Kalman 
				{	
					mu_kalman[0][0] = z_gps[0][0];
					mu_kalman[1][0] = z_gps[1][0];
					first_gps == 1;

				}
				double dT = currentTime.toSec()-previousTime.toSec();
				
				printf("the time : %d - dt : %f - speed : %f - yaw : %f \n es_x : %f - es_y : %f\n" , the_time, dT,currentSpeed,currentYaw,mu_kalman[0][0],mu_kalman[0][1]);

				mu_kk_1[0][0] = Kalman_evalX(mu_kalman[0][0], currentSpeed, currentYaw, (float)dT);
				mu_kk_1[1][0] = Kalman_evalY(mu_kalman[1][0], currentSpeed, currentYaw, (float)dT);

				//P_kk_1 = eval(J)*P*eval(J)' + Q // J is identity matrix
				sum22(Kalman_P,Kalman_Q,P_kk_1);
				

				if (GPS_data_rec > Update_phase)
				{
					substr21(z_gps,mu_kk_1,ybar); //ybar = z - H*mu_kk_1;
					sum22(P_kk_1,Kalman_R,Kalman_S); //S = H*P_kk_1*H'+ R;
					invert22(Kalman_S,Kalman_S_inv); //S^-1
					multip22by22(P_kk_1,Kalman_S_inv,Kalman_K); //K = P_kk_1*H'*S^(-1)
					multip22by21(Kalman_K,ybar,Kalman_K_ybar); //K*ybar;
					sum21(mu_kk_1,Kalman_K_ybar,mu_kalman); //mu_kalman = mu_kk_1 + K*ybar;
					substr22(Kalman_eye,Kalman_K,Kalman_eye_min_K);//(eye(2)-K*H)
					multip22by22(Kalman_eye_min_K,P_kk_1,Kalman_P);//P = (eye(2)-K*H)*P_kk_1;
					Update_phase = GPS_data_rec;
				}

				else{
					equal21(mu_kk_1,mu_kalman);
					equal22(P_kk_1,Kalman_P);
				}
				
			}
		
			/*******************************************/
			/*            MESSAGING SECTION            */
			/*******************************************/

			//save values into msg container
			rem_msg.header.stamp = ros::Time::now();
			rem_msg.temperature = desired_speed;
			rem_msg.variance = desired_roll;

			//save values into msg container for the control readings
			ctrl_msg.header.stamp = ros::Time::now();
			ctrl_msg.temperature = currentSpeed;
			ctrl_msg.variance = currentRoll;

			//save the position readings and the one kalman filtered
			pos_msg.header.stamp = ros::Time::now();
			pos_msg.orientation.x = z_gps[0][0];
			pos_msg.orientation.y = z_gps[1][0];
			pos_msg.orientation.z = mu_kalman[0][0];
			pos_msg.orientation.w = mu_kalman[1][0];

			//publish messages
			remote_pub.publish(rem_msg);
			control_pub.publish(ctrl_msg);
			position_pub.publish(pos_msg);

			/*******************************************/
			/*            LOOPING SECTION              */
			/*******************************************/

			ros::spinOnce();

			loop_rate.sleep();

		}


	  	return 0;
	}

