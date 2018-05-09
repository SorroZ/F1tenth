/*
	teleop_traxxas.cpp 
	purpose: Take in values from user through stdin

	@version: 2.0
	@author: pennARC f1/10 
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/Num.h"
#include "beginner_tutorials/driveCmd.h" // include the custom message defintion 
#include "beginner_tutorials/driveMessage.h"
#include "std_msgs/Float32.h"

#include <unistd.h>
#include <stdio.h> 
#include <termios.h>

struct termios initial_settings, new_settings;
unsigned int msec;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"teleop_traxxas");
	ros::NodeHandle n;

	//Declare the publisher that publishes messages of the custom type - driveCmd
	ros::Publisher pub_teleop = n.advertise<beginner_tutorials::driveCmd>("teleop_commands",1000);

	//The variable that has to be published
	beginner_tutorials::driveCmd tele_cmd;

	//Declare the base default
	float turn = 0;
	float steer_default = 0;
	float heading = 1; // Both these values should correspond to the mid pt value - 9830
	float heading_default = 0;

	float old_throttle_state = 0;
	float current_throttle_state = 0.0;

	float old_steer_state = 0;
	float current_steer_state = 0;

	tcgetattr(0,&initial_settings); 
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON; 
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 0;
	new_settings.c_cc[VTIME] = 1; //Timeout for nonCannonical read in deciseconds
 
	tcsetattr(0, TCSANOW, &new_settings);
	
	int ch, key;
	int flag_st_press, flag_th_press;

	tele_cmd.throttle = 0; 
	tele_cmd.steering = 0; 

	while(ros::ok())
	{
		flag_st_press = 0;
		flag_th_press = 0;

		ch = getchar();

		// 97: Input a from keyboard = turn left	
		if (ch == 97){
			if (tele_cmd.steering != -5) tele_cmd.steering--; 			 
		}
		// 100: Input d from keyboard = turn right 
		else if (ch == 100){
			if (tele_cmd.steering != 5) tele_cmd.steering++;
		}
		// 115: Input s from keyboard = decrease throttle		
		else if (ch == 115){
			if (tele_cmd.throttle != -5) tele_cmd.throttle--;
		}
		// 119: Input w from keyboard = increase throttle
		else if (ch == 119){	
			if (tele_cmd.throttle != 5) tele_cmd.throttle++;
		}	

                ROS_INFO("Steering: %f; Throttle %f ", tele_cmd.steering, tele_cmd.throttle);
                pub_teleop.publish(tele_cmd);
		ros::spinOnce();
		usleep(1000);

    	}

	tcsetattr(0, TCSANOW, &initial_settings);
	return(0);


}


