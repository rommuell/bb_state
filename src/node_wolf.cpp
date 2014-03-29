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
  float leftwheelthrottle;
  float rightwheelthrottle;
  float vpassedthrottle;
  float wpassedthrottle;
  float phifrontwheelthrottle;
  double maxvelocity;





  void joystickCallback(const sensor_msgs::Joy msg2)
  {
    leftwheelthrottle = msg2.axes[1];
    rightwheelthrottle = msg2.axes[4];

    ROS_INFO("left: %f, right: %f", leftwheelthrottle, rightwheelthrottle);
  }




};
int main(int argc, char **argv){

  nodeforcircle node2;
  node2.maxvelocity = 0.3;
  ROS_INFO("Ros MAIN Called again???");
  //Subscriber for Joysticks Joy message:
  ros::init(argc, argv, "nodeforcircle");
  ros::NodeHandle n2;
  ros::Subscriber joysticksub = n2.subscribe("joy", 1, &nodeforcircle::joystickCallback, &node2);
  ros::Publisher FromJoysticktoEpos_pub = n2.advertise<EposManager::EPOSControl>("/motors/BeachBot/Motor_Control", 10);

  EposManager::EPOSControl Eposmsg;

  int count = 0;
  uint64_t IOMessage;
  uint64_t IOStatus;

    //Open Device File for writing to the IO board
  FILE * deviceFilePointer;
  deviceFilePointer = fopen ("/dev/beachbot.ioboard", "w");
  if(deviceFilePointer == NULL){ 
    ROS_ERROR("NO DEVICE FOUND!");
    return 0;
  }
  
  double vpassed;
  double wpassed;
  double vleftwheel = 0;
  double vrightwheel = 0;
  const double beachbotwidthhalf = 0.132;
  double hexvleftwheel;
  double hexvrightwheel;
  double phifrontwheel;
  const double lengthcenterofrotation_frontwheel = 0.31;
  int angleinquants = 0;
  int oldangleinquants = 0;
  int angleinquants1464;
  int angleinquants2928;
  int angleinquantsm1464;
  int angleinquantsm2928;
  int roundsturnedfrontwheelmotor;
  std::vector<int> lorenzvector;


	//frequency regulation (Hz)
  ros::Rate loop_rate(20);
  if (ros::ok()) ROS_INFO("läuft");
	//loop
  while (ros::ok()) {
    ROS_INFO("While begin");
    hexvleftwheel = (32512 * node2.maxvelocity * node2.leftwheelthrottle + 32768);
    hexvrightwheel = (32512 * node2.maxvelocity * node2.rightwheelthrottle + 32768);
		//Joystick input conversion to front wheel steering:
    node2.vpassedthrottle = node2.maxvelocity * (node2.leftwheelthrottle + node2.rightwheelthrottle) / 2;
    node2.wpassedthrottle = node2.maxvelocity * (node2.leftwheelthrottle - node2.rightwheelthrottle) /  (beachbotwidthhalf * 2);
    
    if(node2.vpassedthrottle < 0.00001 && node2.vpassedthrottle > -0.0001) {
      if(node2.wpassedthrottle > 0.00002 || node2.wpassedthrottle < -0.00002) {
        if(node2.wpassedthrottle < 0) {
          node2.phifrontwheelthrottle = -3.14159/2;
        }
        else  {
          node2.phifrontwheelthrottle = 3.14159/2;
        }
      }
      else if (angleinquants == 0) node2.phifrontwheelthrottle = 0;
    }
    else {
      node2.phifrontwheelthrottle = atan2( (lengthcenterofrotation_frontwheel * node2.wpassedthrottle), node2.vpassedthrottle);
    }


 IOStatus = 0x01;

 IOMessage = 0 | (IOStatus<<32) | (((uint64_t)hexvleftwheel)<<16) | (((uint64_t)hexvrightwheel));


 IOMessage = __builtin_bswap64(IOMessage);
 ROS_INFO("leftwheelthrottle: %f, rightwheelthrottle: %f,%d,IOMessage: %#llx,phi: %f" ,node2.leftwheelthrottle ,node2.rightwheelthrottle ,count, IOMessage, node2.phifrontwheelthrottle);

 fwrite (&IOMessage , 8, 1, deviceFilePointer);
 fflush(deviceFilePointer);
 ROS_INFO("WRITE END");
		// Publisher from Joystick to frontwheelEpos:

   angleinquants = floor(node2.phifrontwheelthrottle/3.14159 * 1464);

     roundsturnedfrontwheelmotor = floor(oldangleinquants/1464);

     angleinquants = roundsturnedfrontwheelmotor * 1464 + angleinquants;
     ROS_INFO("anglequants %d", angleinquants);

     lorenzvector.clear();
     lorenzvector.push_back(angleinquants);
     lorenzvector.push_back(angleinquants + 1464);
     lorenzvector.push_back(angleinquants + 2928);
     lorenzvector.push_back(angleinquants - 1464);
     lorenzvector.push_back(angleinquants - 2928);

     int temporary = 300000;

     for(int i=0;i<5;i++)
     {
         if (abs(oldangleinquants - lorenzvector.at(i)) < temporary){
             temporary = abs(oldangleinquants - lorenzvector.at(i));
             angleinquants = lorenzvector.at(i);

         }

     }
    std::cout << angleinquants;







 Eposmsg.node_id = 1;
 Eposmsg.control_mode = 2;
 Eposmsg.setpoint = angleinquants;

 FromJoysticktoEpos_pub.publish(Eposmsg);
		//End of Publisher from Joystick to Eposfrontwheel.
 ROS_INFO("PUBLISH END, Phi in quants %d" , Eposmsg.setpoint);

 oldangleinquants = angleinquants;


 ros::spinOnce();
 loop_rate.sleep();
 count = count + 1;
}

fclose (deviceFilePointer);


return 0;
}
