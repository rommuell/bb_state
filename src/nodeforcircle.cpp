#include "ros/ros.h"
#include "stdint.h"
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseWithCovariance.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "EPOSControl.h"

//generate pointsvectors:
class nodeforcircle
{
  

  
  
public:
/*  
double x_pose;
double y_pose;
 
  
void bot_poseCallback(const geometry_msgs::PoseWithCovariance msg)
{
  //ROS_INFO("Laser Input: [%f]", msg.covariance[1,1]);
  x_pose = msg.covariance[1,1];
  ROS_INFO("Ihear: [%f]",x_pose);
  
  
  
}
*/
public:
float leftwheelthrottle;
float rightwheelthrottle;
float vpassedthrottle;
float wpassedthrottle;
float phifrontwheelthrottle;




void joystickCallback(const sensor_msgs::Joy msg2)
{
  //ROS_INFO("Laser Input: [%f]", msg.covariance[1,1]);
  
  leftwheelthrottle = msg2.axes[1];
  rightwheelthrottle = msg2.axes[4];
  
  //ROS_INFO("Ihear: [%f],[%f]",leftwheelthrottle,rightwheelthrottle);
  
  
  
}




};
int main(int argc, char **argv){

  
    /*
    nodeforcircle node1;

    //Subscriber for Lasers Twist message:
    ros::init(argc, argv, "nodeforcircle");
    ros::NodeHandle n;
    ros::Subscriber lasersub = n.subscribe("bot_pose", 1000, &nodeforcircle::bot_poseCallback, &node1);
    */

    nodeforcircle node2;

    //Subscriber for Joysticks Joy message:
    ros::init(argc, argv, "nodeforcircle");
    ros::NodeHandle n2;

    ros::Subscriber joysticksub = n2.subscribe("joy", 10000, &nodeforcircle::joystickCallback, &node2);
    //ros::Timer timer = n2.createTimer(ros::Duration(0.1), &nodeforcircle::joystickCallback, &node2);


    //ros::Time::init();
    EposManager::EPOSControl Eposmsg;




    uint64_t IOMessage;
    uint64_t IOStatus;

    //Open Device File for writing to the IO board
    FILE * deviceFilePointer;
    deviceFilePointer = fopen ("/dev/beachbot.ioboard", "w");
    //deviceFilePointer = fopen ("/tmp/whatever", "w");

    double vpassed;
    double wpassed;
    double vleftwheel;
    double vrightwheel;
    const double beachbotwidthhalf = 0.132;
    double hexvleftwheel;
    double hexvrightwheel;
    double phifrontwheel;
    const double lengthcenterofrotation_frontwheel = 0.31;
    int angleinquants;
    ros::Publisher FromJoysticktoEpos_pub = n2.advertise<EposManager::EPOSControl>("/motors/BeachBot/Motor_Control", 1000);

	//frequency regulation (Hz)
    ros::Rate loop_rate(200);
	if (ros::ok()) ROS_INFO("läuft");
	ROS_INFO("vleftwheel");
	//loop
	while (ros::ok())
    {
//		if (count>999)
//			{
//			count = 0;
//			}
	
//		vpassed = vvec[count];
//		wpassed = wvec[count];
//		//Wheel velocities
		
//		vleftwheel = vpassed + wpassed * beachbotwidthhalf;
//		vrightwheel = vpassed - wpassed * beachbotwidthhalf;
		
		
		
		
		
		//adjust front wheel steering(sign is mathematically positive) Aufpassen auf singularität!:
		//phifrontwheel = atan((lengthcenterofrotation_frontwheel*wpassed)/vpassed);
		
		
		//convert to hexadecimals:
		/*
		hexvleftwheel = 32512 * vleftwheel + 32768;
		hexvrightwheel = 32512 * vrightwheel + 32768;
		*/
		
		//convert to hexadecimals for joystick (lowered by factor 0.1!!):
		

		
		hexvleftwheel = (32512 * 0.1 * node2.leftwheelthrottle + 32768);
		hexvrightwheel = (32512 * 0.1 * node2.rightwheelthrottle + 32768);
		//Joystick input conversion to front wheel steering:
		node2.vpassedthrottle = 0.1*(node2.leftwheelthrottle+node2.rightwheelthrottle)/2;
		node2.wpassedthrottle = 0.1*(node2.leftwheelthrottle-node2.rightwheelthrottle)/(beachbotwidthhalf*2);
		if(node2.vpassedthrottle < 0.00001 && node2.vpassedthrottle > -0.0001){
			
			if(node2.wpassedthrottle > 0.00002 || node2.wpassedthrottle < -0.02){
				if(node2.wpassedthrottle < 0){
					node2.phifrontwheelthrottle = -3.14159/2;
				}
				else{
					node2.phifrontwheelthrottle = 3.14159/2;
				}
			}
			
		}
		else
		{
			node2.phifrontwheelthrottle = atan((lengthcenterofrotation_frontwheel*node2.wpassedthrottle)/node2.vpassedthrottle);
		}
	
        IOStatus = 0x01;

		IOMessage = 0 | (IOStatus<<32) | (((uint64_t)hexvleftwheel)<<16) | (((uint64_t)hexvrightwheel));
		
	
        ROS_INFO("leftwheelthrottle: %f, rightwheelthrottle: %f,IOMessage: %#llx,phi: %f" ,node2.leftwheelthrottle ,node2.rightwheelthrottle , IOMessage, node2.phifrontwheelthrottle);
				

        IOMessage = __builtin_bswap64(IOMessage);
	
	
		if(deviceFilePointer!=NULL)
			{
			fwrite (&IOMessage , 8, 1, deviceFilePointer);
			fflush(deviceFilePointer);
			}

		// Publisher from Joystick to frontwheelEpos:
		
        angleinquants = floor(node2.phifrontwheelthrottle*2/3.14159 * 1464);
		

		
		Eposmsg.node_id = 1;
		Eposmsg.control_mode = 2;
		Eposmsg.setpoint = angleinquants;
		
		FromJoysticktoEpos_pub.publish(Eposmsg);
		//End of Publisher from Joystick to Eposfrontwheel.
		ROS_INFO("phiinquants %d" , Eposmsg.setpoint);
		
        ros::spinOnce();
		loop_rate.sleep();



	}

    fclose (deviceFilePointer);



    return 0;
}
