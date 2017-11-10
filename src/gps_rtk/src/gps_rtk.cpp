#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include  <stdlib.h>
#include  <assert.h>
#include  <ros/ros.h>
#include  <sensor_msgs/NavSatFix.h>
#include  <string>

void init_gps_msg(sensor_msgs::NavSatFix* gps_msg)
{
	gps_msg->header.stamp = ros::Time::now();
	gps_msg->latitude = 0.0f;
	gps_msg->longitude = 0.0f;
	gps_msg->altitude = 0.0f;

	for(int i=0; i<9; i++)
	{
		gps_msg->position_covariance[i] = !(i%4);
	}

	gps_msg->position_covariance_type = 0;
}

void update_gps_msg(sensor_msgs::NavSatFix* gps_msg, double pos_data[3])
{
	gps_msg->header.stamp = ros::Time::now();
	gps_msg->latitude = pos_data[0];
	gps_msg->longitude = pos_data[1];
	gps_msg->altitude = pos_data[2];

	ROS_INFO("GPS : Lat : = %.8f, Long = %.8f, Alt = %.3f", pos_data[0], pos_data[1], pos_data[2]);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gps_handler");
	ros::NodeHandle n;
	ros::Publisher gps_rtk = n.advertise<sensor_msgs::NavSatFix>("gps_readings", 1000);
	ros::Rate loop_rate(10);
	
	double pos_data[3];
	sensor_msgs::NavSatFix gps_msg;
	init_gps_msg(&gps_msg);
	
	FILE *fp;
	char path[1035];

	fp = popen("nc 192.168.2.15 9001", "r");
	if (fp == NULL)
	{
		printf("Failed to run command\n");
		exit(1);
	}

	while(fgets(path, sizeof(path)-1,fp) != NULL && ros::ok())
	{
		char *str = strdup(path);
		char *token;
		int i = 0;
		while((token = strsep(&str, "   ")))
		{
		  if(i == 4)
		  {
			//pos_data[0] = 0;
			pos_data[0] = strtod(token, NULL);
			//printf("Longitude = %s, ", token);
		  }
		  if(i == 8)
		  {
			//pos_data[2] = 2;
			pos_data[1] = strtod(token, NULL);
			//printf("Latitude = %s, ", token);
		  }
		  if(i == 11)
		  {
			//pos_data[3] = 3;
			pos_data[2] = strtod(token, NULL);
			//printf("Altitude = %s\n", token);
		  }
		  
		  update_gps_msg(&gps_msg, pos_data);
		  gps_rtk.publish(gps_msg);
		  i++;
		}
		free(token);
		free(str);
		loop_rate.sleep();
	}

	pclose(fp);
	return 0;
}
