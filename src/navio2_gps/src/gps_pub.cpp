/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Receive position information with GPS driver onboard of Navio shield for Raspberry Pi.

Ublox GPS receiver is connected as an SPI device 0.0(/dev/spidev0.0).
The receiver sends information over SPI in an endless stream,
this program is intended to show, how to capture ubx protocol messages
from the stream and extract useful information from them.

To run this example navigate to the directory containing it and run following commands:
make
./gps
*/

//#define _XOPEN_SOURCE 600
#include "Ublox.h"
#include "Util.h"

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

using namespace std;

void init_gps_msg(sensor_msgs::NavSatFix* gps_msg)
{
	gps_msg->header.stamp = ros::Time::now();

	gps_msg->latitude = 0.0f;
	gps_msg->longitude = 0.0f;
	gps_msg->altitude = 0.0f;

  //Create a cov matrix of [1 0 0; 0 1 0; 0 0 1]
	for(int i = 0; i < 9; i++)
	{
		gps_msg->position_covariance[i] = !(i%4);
	}

	gps_msg->position_covariance_type = 0;
}

void update_gps_msg(sensor_msgs::NavSatFix* gps_msg, std::vector<double> pos_data, Ublox gps)
{
	gps_msg->header.stamp = ros::Time::now();

	
	if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
    {
		gps_msg->latitude = pos_data[2]/10000000;
		gps_msg->longitude = pos_data[1]/10000000;
		gps_msg->altitude = pos_data[3]/1000;

		ROS_INFO("GPS : Lat = %f, Long = %f, Alt = %f", pos_data[2]/10000000, pos_data[1]/10000000, pos_data[3]/1000);
	}

	else 
	{
	  /*gps_msg->latitude = prev_lat;
	  gps_msg->longitude = prev_long;
	  gps_msg->altitude = prev_alt;*/
      ROS_INFO("Message not captured");
    }


    /*if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1)
    {
        ROS_INFO("Current GPS status:");
        ROS_INFO("gpsFixOk: %d", ((int)pos_data[1] & 0x01));

        ROS_INFO("gps Fix status: ");
        switch((int)pos_data[0]){
            case 0x00:
            	gps_msg->status.status = -1;
                ROS_INFO("no fix");
                break;

            case 0x01:
            	gps_msg->status.status = -1;
                ROS_INFO("dead reckoning only");
                break;

            case 0x02:
            	gps_msg->status.status = -1;
                ROS_INFO("2D-fix");
                break;

            case 0x03:
            	gps_msg->status.status = 0;
                ROS_INFO("3D-fix");
                break;

            case 0x04:
            	gps_msg->status.status = -1;
                ROS_INFO("GPS + dead reckoning combined");
                break;

            case 0x05:
            	gps_msg->status.status = -1;
                ROS_INFO("Time only fix");
                break;

            default:
            	gps_msg->status.status = -1;
                ROS_INFO("Reserved value. Current state unknown");
                break;

        		}
        }


   else 
   {
	  /*gps_msg->latitude = prev_lat;
	  gps_msg->longitude = prev_long;
	  gps_msg->altitude = prev_alt;  	
      ROS_INFO("Status Message not captured");
   }*/

}

int main(int argc, char *argv[]){

	ros::init(argc, argv, "gps_handler");
	ros::NodeHandle n;
	ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_readings", 1000);

	ros::Rate loop_rate(2);

    if (check_apm()) {
        return 1;
    }

    // This vector is used to store location data, decoded from ubx messages.
    // After you decode at least one message successfully, the information is stored in vector
    // in a way described in function decodeMessage(vector<double>& data) of class UBXParser(see ublox.h)

    std::vector<double> pos_data;


    // create ublox class instance
    Ublox gps;

    // Here we test connection with the receiver. Function testConnection() waits for a ubx protocol message and checks it.
    // If there's at least one correct message in the first 300 symbols the test is passed
    if(gps.testConnection())
    {
        printf("Ublox test OK\n");

        // gps.decodeMessages();
        // You can use this function to decode all messages, incoming from the GPS receiver. The function starts an infinite loop.
        // In this example we can only decode NAV_STATUS and NAV-POSLLH messages, the others are simply ignored.
        // You can add new message types in function decodeMessage() of class UBXParser(see ublox.h)


        // Here, however we use a different approach. Instead of trying to extract info from every message(as done in decodeMessages()),
        // this function waits for a message of a specified type and gets you just the information you need
        // In this example we decode NAV_STATUS and NAV-POSLLH messages, adding new types, however is quite easy

        sensor_msgs::NavSatFix gps_msg;
        init_gps_msg(&gps_msg);	

        while (ros::ok())
        {	

            update_gps_msg(&gps_msg, pos_data, gps);

            gps_pub.publish(gps_msg);

            ros::spinOnce();

            loop_rate.sleep();
            // after desired message is successfully decoded, we can use the information stored in pos_data vector
            // right here, or we can do something with it from inside decodeSingleMessage() function(see ublox.h).
            // the way, data is stored in pos_data vector is specified in decodeMessage() function of class UBXParser(see ublox.h)
            /*printf("GPS Millisecond Time of Week: %.0lf s\n", pos_data[0]/1000);
            printf("Longitude: %lf\n", pos_data[1]/10000000);
            printf("Latitude: %lf\n", pos_data[2]/10000000);
            printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
            printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
            printf("Horizontal Accuracy Estateimate: %.3lf m\n", pos_data[5]/1000);
	        printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);*/

            //usleep(200);
        }

    } else {

        ROS_INFO("Ublox test not passed\nAbort program!\n");

    }

    return 0;
}
